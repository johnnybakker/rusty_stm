#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [EXTI0])]
mod app {
	
	use stm32h7xx_hal::prelude::*;
    use cortex_m_semihosting::{hprintln};
    use rtic_monotonics::systick::*;
	use stm32h7xx_hal::gpio::PC13;
	use stm32h7xx_hal::gpio::Output;
	use stm32h7xx_hal::gpio::PushPull;
	use stm32h7xx_hal::rcc::PllConfigStrategy;
	use stm32h7xx_hal::ethernet::{ EthernetDMA, EthernetMAC, DesRing, PHY, phy::LAN8742A };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
		led: PC13<Output<PushPull>>,
		adapter: LAN8742A<EthernetMAC>
	}

	/// Ethernet descriptor rings are a global singleton
	#[link_section = ".sram3.eth"]
	static mut DES_RING: DesRing<4,4> = DesRing::new();

	const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
    
		let pwr = ctx.device.PWR.constrain();
		let pwrcfg = pwr.vos0(&ctx.device.SYSCFG).freeze();

		let rcc = ctx.device.RCC.constrain();

		let ccdr = rcc
			.use_hse(25.MHz())
			.sys_ck(480.MHz())
			.pll1_strategy(PllConfigStrategy::Fractional)
			.pll1_p_ck(480.MHz())
			.pll1_q_ck(480.MHz())
			.pll1_r_ck(480.MHz())
			.pll2_strategy(PllConfigStrategy::Fractional)
			.pll2_p_ck(240.MHz())
			.pll2_q_ck(30.MHz())
			.pll2_r_ck(240.MHz())
			.pll3_strategy(PllConfigStrategy::Fractional)
			.pll3_p_ck(50.MHz())
			.pll3_q_ck(50.MHz())
			.pll3_r_ck(50.MHz())
			.mco1_from_hsi(64.MHz())
			.freeze(pwrcfg, &ctx.device.SYSCFG);

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, ccdr.clocks.sysclk().raw(), systick_token);

		hprintln!("init {}", ccdr.clocks.sysclk().raw());
        
		let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
		let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);

		let mac_address = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);
   
        let adapter = unsafe {

			let (dma, mac) = stm32h7xx_hal::ethernet::new(
				ctx.device.ETHERNET_MAC,
				ctx.device.ETHERNET_MTL,
				ctx.device.ETHERNET_DMA,
				(
					gpioa.pa1.into_alternate(),  // REF_CLK
					gpioa.pa2.into_alternate(),  // MDIO
					gpioc.pc1.into_alternate(),  // MDC
					gpioa.pa7.into_alternate(),  // CRS_DV
					gpioc.pc4.into_alternate(),  // RX0
					gpioc.pc5.into_alternate(),  // RX1
					gpiob.pb11.into_alternate(), // TX_EN
					gpiob.pb12.into_alternate(), // TX0
					gpiob.pb13.into_alternate(), // TX1
				),
				&mut DES_RING,
				mac_address.clone(),
				ccdr.peripheral.ETH1MAC,
				&ccdr.clocks,
			);
	
			//Initialise ethernet PHY...
			let mut lan8742a = LAN8742A::new(mac.set_phy_addr(1));
			lan8742a.phy_reset();
			lan8742a.phy_init();

			// Enable ETH interrupts
			stm32h7xx_hal::ethernet::enable_interrupt(); 

	
			lan8742a
		};


        (Shared {}, Local { led: gpioc.pc13.into(), adapter })
    }

	#[idle]
	fn idle(_cx: idle::Context) -> ! {

		adapter_status::spawn().ok();

		loop { }
	}

    #[task(local = [led, adapter], priority=1)]
    async fn adapter_status(ctx: adapter_status::Context) {
		loop {
			
			match ctx.local.adapter.poll_link() {
                true => ctx.local.led.set_low(),
                _ => ctx.local.led.set_high(),
            }

		}
    }

	#[task(binds = ETH)]
    fn ethernet_event(_ctx: ethernet_event::Context) {
        unsafe { stm32h7xx_hal::ethernet::interrupt_handler() }

		
		// ctx.shared.interface.lock(|interface|{
		// 	interface.poll(Instant::from_millis(time)).unwrap();


		// 	handle_socket::spawn().unwrap()
		// });
    }


	#[panic_handler]
	fn panic(info: &core::panic::PanicInfo) -> ! {
		hprintln!("Error {}", info);
		loop {	}
	}
}