#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [EXTI0])]
mod app {
	
	use rusty_stm32::prelude::Net;
	use rusty_stm32::version::VERSION;
	use rusty_stm32::version::VERSION_BRANCH;
	use rusty_stm32::version::VERSION_MINUTE;
	use smoltcp::socket::TcpSocket;
	use smoltcp::socket::TcpSocketBuffer;
	use stm32h7xx_hal::prelude::*;
    use rtic_monotonics::systick::*;
    use rtic_monotonics::*;
	use stm32h7xx_hal::gpio::PC13;
	use stm32h7xx_hal::gpio::Output;
	use stm32h7xx_hal::gpio::PushPull;
	use stm32h7xx_hal::rcc::PllConfigStrategy;
	use stm32h7xx_hal::ethernet::{ EthernetDMA, EthernetMAC, DesRing, PHY, phy::LAN8742A };
	use core::fmt::Write;
	use core::write;


    #[shared]
    struct Shared {
		interface: smoltcp::iface::Interface<'static, EthernetDMA<'static, 4, 4>>,
		requests: u32
	}

    #[local]
    struct Local {
		led: PC13<Output<PushPull>>,
		adapter: LAN8742A<EthernetMAC>
	}

	// /// Ethernet descriptor rings are a global singleton
	#[link_section = ".sram3.eth"]
	static mut DES_RING: DesRing<4,4> = DesRing::new();

	const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];

    #[init(local = [
		net: Net<'static> = Net::new(),
		tx: [u8; 512] = [0; 512],
		rx: [u8; 512] = [0; 512],
		des_ring: DesRing<4,4> = DesRing::new()
	])]
    fn init(ctx: init::Context) -> (Shared, Local) {

		let pwr = ctx.device.PWR.constrain();
		let pwrcfg = pwr.vos0(&ctx.device.SYSCFG).freeze();

		let rcc = ctx.device.RCC.constrain();

		let ccdr = rcc
			.use_hse(25.MHz())
			.sys_ck(480.MHz())
			.sysclk(480.MHz())
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
        
		let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
		let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);

		let tx = gpioa.pa9.into_alternate();
		let rx = gpioa.pa10.into_alternate();
		let serial = ctx.device.USART1.serial((tx, rx), 115200.bps(), ccdr.peripheral.USART1, &ccdr.clocks).unwrap();
		let (mut tx, mut _rx) = serial.split();
		writeln!(tx, "Hello, world!\r").unwrap();
		writeln!(tx, "Version: {} {:?}", VERSION_BRANCH, VERSION);
		writeln!(tx, "init {}", ccdr.clocks.sysclk().raw());

		let mac_address = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);
   
        let (adapter, dma) = unsafe {

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

	
			(lan8742a, dma)
		};
		
		let ip = [192,168,178,100];
		let interface = ctx.local.net.create_interface(dma, mac_address.into(), &ip);

		tcp_socket::spawn(ctx.local.rx, ctx.local.tx).unwrap();

        (Shared { interface, requests: 0 }, Local { led: gpioc.pc13.into(), adapter })
    }

	#[idle]
	fn idle(_cx: idle::Context) -> ! {

		adapter_status::spawn().ok();

		loop {
			//rtic::export::wfi()
		}
	}

	#[task(shared = [interface, requests], priority = 1)]
	async fn tcp_socket(mut ctx: tcp_socket::Context, tx: &'static mut [u8], rx: &'static mut [u8]) {

		let handle = ctx.shared.interface.lock(|interface|{
			let socket = TcpSocket::new(
				TcpSocketBuffer::new(&mut rx[..]), 
				TcpSocketBuffer::new(&mut tx[..])
			);

			interface.add_socket(socket)
		});

		
		loop {
	
			(&mut ctx.shared.interface, &mut ctx.shared.requests).lock(|interface, requests|{

				let socket: &mut TcpSocket = interface.get_socket(handle);

				if !socket.is_listening() && !socket.is_open() {
					socket.close();
					socket.listen(80).unwrap();
				} 

				if socket.recv_queue() != 0 {
					let mut data = [0; 1024];
					let size = socket.recv_slice(&mut data).unwrap();
					let message = core::str::from_utf8(&data[0..size]).unwrap();
					let mut parts = message.split("\r\n");
				
					if let Some(line) = parts.next() {
						let mut line = line.split(' ');
						
						let method = line.next();
						let path = line.next();
						let _version = line.next();

						if let (Some(method), Some(path)) = (method, path) {
							if method == "GET" {
								if path == "/request" {
									*requests+=1;
								}
								write!(socket, "Requests: {}", requests).unwrap();
							} 
						}
					} 

					socket.close();
				} 

			});

			Systick::delay(1.millis()).await;
		}
	}

    #[task(local = [led, adapter], priority=1)]
    async fn adapter_status(ctx: adapter_status::Context) {
		loop {
			
			match ctx.local.adapter.poll_link() {
                true => ctx.local.led.set_low(),
                _ => ctx.local.led.set_high(),
            }

			Systick::delay(100.millis()).await;
		}
    }

	#[task(binds = ETH, shared = [interface])]
    fn ethernet_event(mut ctx: ethernet_event::Context) {
        unsafe { stm32h7xx_hal::ethernet::interrupt_handler() }

		ctx.shared.interface.lock(|interface|{
			let now = Systick::now().ticks();
			interface.poll(smoltcp::time::Instant::from_millis(now)).unwrap();
		});
    }


	#[panic_handler]
	fn panic(info: &core::panic::PanicInfo) -> ! {
		loop {	}
	}
}
