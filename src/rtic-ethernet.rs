#![no_std]
#![no_main]


// #[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
// mod app {

//     use stm32h7xx_hal::device::flash::crcdatar;
//     use stm32h7xx_hal::ethernet::{PinsRMII, EthernetDMA};
//     use stm32h7xx_hal::gpio::gpioc::{PC13};
//     use stm32h7xx_hal::gpio::{Output, PushPull};
// 	use stm32h7xx_hal::stm32::TIM1;
//     use stm32h7xx_hal::rcc::PllConfigStrategy;
// 	use stm32h7xx_hal::time::MilliSeconds;
//     use stm32h7xx_hal::timer::{Event, Timer};
//     use stm32h7xx_hal::prelude::*;
// 	use panic_halt as _;
// 	use systick_monotonic::Systick;
// 	use core::fmt::Write;

// 	use smoltcp::iface::{InterfaceBuilder, NeighborCache};
// 	use smoltcp::socket::{TcpSocket, TcpSocketBuffer};
// 	use smoltcp::time::Instant;
// 	use smoltcp::wire::{IpAddress, IpCidr, Ipv4Address, Ipv4Cidr};
// 	use smoltcp::iface::{Interface, Neighbor, SocketStorage, Route};
// 	use core::sync::atomic::AtomicU32;

// 	use stm32h7xx_hal::{ethernet, ethernet::PHY};

// 	#[link_section = ".sram3.eth"]
// 	static mut DES_RING: ethernet::DesRing<4, 4> = ethernet::DesRing::new();

// 	const IP_ADDRESS: Ipv4Address = Ipv4Address::new(192, 168, 100, 200);
// 	const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];

//     #[shared]
//     struct SharedResources {}

//     #[local]
//     struct LocalResources {
// 		timer1: Timer<TIM1>,
//         led: PC13<Output<PushPull>>,
// 		interface: Interface<'static, EthernetDMA<'static, 4, 4>>,
//         tcp_handle: smoltcp::iface::SocketHandle,
//     }

// 	#[monotonic(binds = SysTick, default = true)]
//     type Monotonic = Systick<1000>;

//     fn now_fn() -> smoltcp::time::Instant {
//         let time = monotonics::now().duration_since_epoch().ticks();
//         smoltcp::time::Instant::from_millis(time as i64)
//     }


// 	/// TIME is an atomic u32 that counts milliseconds.
// 	static TIME: AtomicU32 = AtomicU32::new(0);	

// 	fn systick_init(mut syst: stm32::SYST, clocks: stm32h7xx_hal::rcc::CoreClocks) {
// 		let c_ck_mhz = clocks.c_ck().to_MHz();
	
// 		let syst_calib = 0x3E8;
	
// 		syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
// 		syst.set_reload((syst_calib * c_ck_mhz) - 1);
// 		syst.enable_interrupt();
// 		syst.enable_counter();
// 	}



	

//     #[init]
//     fn init(ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {	
		
// 		// Constrain and Freeze power
// 		let pwrcfg = ctx.device.PWR.constrain().vos0(&ctx.device.SYSCFG).freeze();
		
// 		// Constrain and Freeze clock
// 		let rcc = ctx.device.RCC.constrain();
		
// 		let ccdr = rcc
// 			.use_hse(25.MHz())
// 			.sys_ck(480.MHz())
// 			.pll1_strategy(PllConfigStrategy::Fractional)
// 			.pll1_p_ck(480.MHz())
// 			.pll1_q_ck(480.MHz())
// 			.pll1_r_ck(480.MHz())
// 			.pll2_strategy(PllConfigStrategy::Fractional)
// 			.pll2_p_ck(240.MHz())
// 			.pll2_q_ck(30.MHz())
// 			.pll2_r_ck(240.MHz())
// 			.pll3_strategy(PllConfigStrategy::Fractional)
// 			.pll3_p_ck(50.MHz())
// 			.pll3_q_ck(50.MHz())
// 			.pll3_r_ck(50.MHz())
// 			.mco1_from_hsi(64.MHz())
// 			.freeze(pwrcfg, &ctx.device.SYSCFG);

// 		let mono = Systick::new(ctx.core.SYST, ccdr.clocks.hclk().raw());

// 		let mut timer1 = ctx.device.TIM1.timer(
// 			MilliSeconds::millis(1000).into_rate(),
// 			ccdr.peripheral.TIM1,
// 			&ccdr.clocks,
// 		);

// 		timer1.listen(Event::TimeOut);

// 		let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
// 		let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
// 		let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);

// 		let tx = gpioa.pa9.into_alternate();
// 		let rx = gpioa.pa10.into_alternate();
// 		let serial = ctx.device.USART1.serial((tx, rx), 115200.bps(), ccdr.peripheral.USART1, &ccdr.clocks).unwrap();

// 		let (mut tx, mut _rx) = serial.split();
// 		writeln!(tx, "Hello, world!\r").unwrap();

// 		let rmii_ref_clk = gpioa.pa1.into_alternate();
// 		let rmii_mdio = gpioa.pa2.into_alternate();
// 		let rmii_mdc = gpioc.pc1.into_alternate();
// 		let rmii_crs_dv = gpioa.pa7.into_alternate();
// 		let rmii_rxd0 = gpioc.pc4.into_alternate();
// 		let rmii_rxd1 = gpioc.pc5.into_alternate();
// 		let rmii_tx_en = gpiob.pb11.into_alternate();
// 		let rmii_txd0 = gpiob.pb12.into_alternate();
// 		let rmii_txd1 = gpiob.pb13.into_alternate();

// 		let pinsRmmi = (
// 			rmii_ref_clk, rmii_mdio, rmii_mdc, rmii_crs_dv, 
// 			rmii_rxd0, rmii_rxd1, rmii_tx_en, rmii_txd0, rmii_txd1);

// 		let mac_address = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);

// 		let (dma, mac) = ethernet::new(
// 			ctx.device.ETHERNET_MAC, ctx.device.ETHERNET_MTL, ctx.device.ETHERNET_DMA,
// 			pinsRmmi, unsafe { &mut DES_RING }, mac_address.clone(), ccdr.peripheral.ETH1MAC,
// 			&ccdr.clocks);
		
// 		//Initialise ethernet PHY...
// 		let mut lan8742a = ethernet::phy::LAN8742A::new(mac.set_phy_addr(1));
// 		lan8742a.phy_reset();
// 		lan8742a.phy_init();


// 		writeln!(tx, "Ethernet\r").unwrap();

// 		let ip_addr = IpCidr::new(IpAddress::from(IP_ADDRESS), 24);
// 		let mut ip_addrs = [ip_addr];
// 		let mut neighbor_storage = [None; 16];

// 		let neighbor_cache = NeighborCache::new(&mut neighbor_storage[..]);

// 		let mut sockets: [_; 1] = Default::default();
// 		let mut interface = InterfaceBuilder::new(dma, &mut sockets[..])
// 			.hardware_addr(mac_address.into())
// 			.ip_addrs(&mut ip_addrs[..])
// 			.neighbor_cache(neighbor_cache)
// 			.finalize();

// 		let mut server_rx_buffer = [0; 512];
// 		let mut server_tx_buffer = [0; 512];
    
// 		let server_socket = TcpSocket::new(
// 			TcpSocketBuffer::new(&mut server_rx_buffer[..]),
// 			TcpSocketBuffer::new(&mut server_tx_buffer[..]),
// 		);
    
// 		let tcp_handle = interface.add_socket(server_socket);
		
// 		let shared = SharedResources {};
		
// 		let local = LocalResources {
// 			timer1,
// 			led: gpioc.pc13.into_push_pull_output(),
// 			interface,
// 			tcp_handle
// 		};

//         (shared, local, init::Monotonics(mono))
//     }

//     #[task(binds = TIM1_UP, local = [led, timer1])]
//     fn timer1_tick(ctx: timer1_tick::Context) {
//         ctx.local.timer1.clear_irq();
//         ctx.local.led.toggle();
//     }

// 	// #[task(binds = ETH, local = [interface, tcp_handle, dma, sockets, data: [u8; 512] = [0u8; 512]], priority = 2)]
//     // fn eth_interrupt(cx: eth_interrupt::Context) {
//     //     let (iface, tcp_handle, buffer, sockets, mut dma) = (
//     //         cx.local.interface,
//     //         cx.local.tcp_handle,
//     //         cx.local.data,
//     //         cx.local.sockets,
//     //         cx.local.dma,
//     //     );

//     //     let interrupt_reason = stm32_eth::eth_interrupt_handler();
//     //     defmt::debug!("Got an ethernet interrupt! Reason: {}", interrupt_reason);

//     //     iface.poll(now_fn(), &mut dma, sockets);

//     //     let socket = sockets.get_mut::<TcpSocket>(*tcp_handle);
//     //     if let Ok(recv_bytes) = socket.recv_slice(buffer) {
//     //         if recv_bytes > 0 {
//     //             socket.send_slice(&buffer[..recv_bytes]).ok();
//     //             defmt::info!("Echoed {} bytes.", recv_bytes);
//     //         }
//     //     }

//     //     if !socket.is_listening() && !socket.is_open() || socket.state() == TcpState::CloseWait {
//     //         socket.abort();
//     //         socket.listen(crate::SOCKET_ADDRESS).ok();
//     //         defmt::warn!("Disconnected... Reopening listening socket.");
//     //     }

//     //     iface.poll(now_fn(), &mut dma, sockets);
//     // }

// }