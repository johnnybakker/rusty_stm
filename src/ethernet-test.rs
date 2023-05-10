#![no_main]
#![no_std]

use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::SYST;
use cortex_m_rt::{entry, exception};
use smoltcp::iface::{InterfaceBuilder, NeighborCache};
use smoltcp::socket::{TcpSocket, TcpSocketBuffer};
use smoltcp::time::Instant;
use smoltcp::wire::{IpAddress, IpCidr, Ipv4Address};
use stm32h7xx_hal::interrupt;
use stm32h7xx_hal::{pac, prelude::*, rcc::PllConfigStrategy};
use stm32h7xx_hal::{ethernet, ethernet::PHY};
use panic_halt as _;

use core::cell::RefCell;
use core::fmt::Write;
use core::sync::atomic::{AtomicU32, Ordering};

// TX PA9
// RX PA10
// LED PC13

/// Ethernet descriptor rings are a global singleton
#[link_section = ".sram3.eth"]
static mut DES_RING: ethernet::DesRing<4, 4> = ethernet::DesRing::new();

/// Locally administered MAC address
//const IP_ADDRESS: Ipv4Address = Ipv4Address::new(192, 168, 178, 100);
const IP_ADDRESS: Ipv4Address = Ipv4Address::new(192, 168, 100, 200);
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];

static TIME: AtomicU32 = AtomicU32::new(0);
static ETH_PENDING: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));


#[entry]
fn main() -> ! {

    let mut cp = cortex_m::Peripherals::take().unwrap();

    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwrcfg = dp.PWR.constrain().vos0(&dp.SYSCFG).freeze();
	
    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
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
		.freeze(pwrcfg, &dp.SYSCFG);

	cp.SCB.enable_icache();
    cp.DWT.enable_cycle_counter();
	
	cp.SYST.set_reload(SYST::get_ticks_per_10ms() / 10);
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();

	let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

	let tx = gpioa.pa9.into_alternate();
    let rx = gpioa.pa10.into_alternate();


	let serial = dp
		.USART1
		.serial((tx, rx), 115200.bps(), ccdr.peripheral.USART1, &ccdr.clocks)
		.unwrap();

	let (mut tx, mut _rx) = serial.split();
	writeln!(tx, "Hello, world!\r").unwrap();

    let mut led = gpioc.pc13.into_push_pull_output();
	led.set_low();


    let rmii_ref_clk = gpioa.pa1.into_alternate();
	let rmii_mdio = gpioa.pa2.into_alternate();
    let rmii_mdc = gpioc.pc1.into_alternate();
    let rmii_crs_dv = gpioa.pa7.into_alternate();
    let rmii_rxd0 = gpioc.pc4.into_alternate();
    let rmii_rxd1 = gpioc.pc5.into_alternate();
    let rmii_tx_en = gpiob.pb11.into_alternate();
    let rmii_txd0 = gpiob.pb12.into_alternate();
    let rmii_txd1 = gpiob.pb13.into_alternate();


	let mac_address = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);


    let (device, eth_mac) = unsafe {
        ethernet::new(
            dp.ETHERNET_MAC,
            dp.ETHERNET_MTL,
            dp.ETHERNET_DMA,
            (
                rmii_ref_clk,
                rmii_mdio,
                rmii_mdc,
                rmii_crs_dv,
                rmii_rxd0,
                rmii_rxd1,
                rmii_tx_en,
                rmii_txd0,
                rmii_txd1,
            ),
            &mut DES_RING,
			mac_address.clone(),
            ccdr.peripheral.ETH1MAC,
            &ccdr.clocks,
        )
    };

	unsafe {
        ethernet::enable_interrupt();
        cp.NVIC.set_priority(stm32h7xx_hal::stm32::Interrupt::ETH, 196); // Mid prio
        cortex_m::peripheral::NVIC::unmask(stm32h7xx_hal::stm32::Interrupt::ETH);
    };

	
    //Initialise ethernet PHY...
    let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(1));

   	lan8742a.phy_reset();
   	lan8742a.phy_init();



    // Get the delay provider.
    //let mut delay = cp.SYST.delay(ccdr.clocks);

	writeln!(tx, "Ethernet\r").unwrap();



    let ip_addr = IpCidr::new(IpAddress::from(IP_ADDRESS), 24);
    let mut ip_addrs = [ip_addr];
    let mut neighbor_storage = [None; 16];
    let neighbor_cache = NeighborCache::new(&mut neighbor_storage[..]);

    let mut sockets: [_; 1] = Default::default();
    let mut iface = InterfaceBuilder::new(device, &mut sockets[..])
        .hardware_addr(mac_address.into())
        .ip_addrs(&mut ip_addrs[..])
        .neighbor_cache(neighbor_cache)
        .finalize();

    let mut server_rx_buffer = [0; 512];
    let mut server_tx_buffer = [0; 512];
    
	let server_socket = TcpSocket::new(
        TcpSocketBuffer::new(&mut server_rx_buffer[..]),
        TcpSocketBuffer::new(&mut server_tx_buffer[..]),
    );
    
	let server_handle = iface.add_socket(server_socket);


    let mut eth_up = false;
	let mut last_send_millis = 0;

    loop {
        // Ethernet
        let eth_last = eth_up;
        eth_up = lan8742a.poll_link();

        match eth_up {
            true => led.set_low(),
            _ => led.set_high(),
        }

        if eth_up != eth_last {
            // Interface state change
            match eth_up {
                true => writeln!(tx, "Ethernet UP").unwrap(),
                _ => writeln!(tx, "Ethernet DOWN").unwrap(),
            }
        }

		let time = TIME.load(Ordering::SeqCst);
        
		cortex_m::interrupt::free(|cs| {
            let mut eth_pending = ETH_PENDING.borrow(cs).borrow_mut();
            *eth_pending = false;
        });
	
      	if iface.poll(Instant::from_millis(time as i64)).is_err() {
			writeln!(tx, "Failed to poll from socket").unwrap();
		}

      	let socket = iface.get_socket::<TcpSocket>(server_handle);

        if !socket.is_listening() && !socket.is_open() {
            
			socket.abort();
            
			if let Err(e) = socket.listen(80) {
                writeln!(tx, "TCP listen error: {:?}", e).unwrap();
            } else {
                writeln!(tx, "Listening at {}:80...", ip_addr).unwrap();
            }


        } else if socket.send_queue() == 0 {  

			
			match socket.send_slice(b"hello\r\n") {
				Ok(_) => {
					writeln!(tx, "Transmitted hello! Closing socket...").unwrap();
					//last_send_millis = time;
				}
				Err(_) => {}
			}
		
			

        }





    }
}

#[exception]
fn SysTick() {
	TIME.fetch_and(1, Ordering::SeqCst);
}

#[interrupt]
fn ETH() {

	cortex_m::interrupt::free(|cs| {
        let mut eth_pending = ETH_PENDING.borrow(cs).borrow_mut();
        *eth_pending = true;
    });

    unsafe { 
		ethernet::interrupt_handler() 
	}
}