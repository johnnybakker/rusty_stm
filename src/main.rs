#![no_main]
#![no_std]

use cortex_m_rt::{entry};
use smoltcp::phy::{Device, RxToken, TxToken};
use stm32h7xx_hal::ethernet::EthernetMAC;
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::interrupt;
use stm32h7xx_hal::{pac, prelude::*, rcc::PllConfigStrategy};
use stm32h7xx_hal::{ethernet, ethernet::PHY, stm32::ETHERNET_MAC};
use stm32h7xx_hal::ethernet::phy::LAN8742A;
use panic_abort as _;

use core::fmt::Write;

// TX PA9
// RX PA10
// LED PC13

/// Locally administered MAC address
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];

/// Ethernet descriptor rings are a global singleton
#[link_section = ".sram3.eth"]
static mut DES_RING: ethernet::DesRing<4, 4> = ethernet::DesRing::new();

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
    // TODO: ETH DMA coherence issues
    // cp.SCB.enable_dcache(&mut cp.CPUID);
    cp.DWT.enable_cycle_counter();


	let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioi = dp.GPIOI.split(ccdr.peripheral.GPIOI);

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


    let mut rmii_ref_clk = gpioa.pa1.into_alternate();
    let mut rmii_mdio = gpioa.pa2.into_alternate();
    let mut rmii_mdc = gpioc.pc1.into_alternate();
    let mut rmii_crs_dv = gpioa.pa7.into_alternate();
    let mut rmii_rxd0 = gpioc.pc4.into_alternate();
    let mut rmii_rxd1 = gpioc.pc5.into_alternate();
    let mut rmii_tx_en = gpiob.pb11.into_alternate();
    let mut rmii_txd0 = gpiob.pb12.into_alternate();
    let mut rmii_txd1 = gpiob.pb13.into_alternate();

	rmii_ref_clk.set_speed(Speed::VeryHigh);
	rmii_mdio.set_speed(Speed::VeryHigh);
	rmii_mdc.set_speed(Speed::VeryHigh);
	rmii_crs_dv.set_speed(Speed::VeryHigh);
	rmii_rxd0.set_speed(Speed::VeryHigh);
	rmii_rxd1.set_speed(Speed::VeryHigh);
	rmii_tx_en.set_speed(Speed::VeryHigh);
	rmii_txd0.set_speed(Speed::VeryHigh);
	rmii_txd1.set_speed(Speed::VeryHigh);

	let mac_address = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);

    let (mut eth_dma, eth_mac) = unsafe {
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
			mac_address,
            ccdr.peripheral.ETH1MAC,
            &ccdr.clocks,
        )
    };

	unsafe {
        ethernet::enable_interrupt();
        cp.NVIC.set_priority(stm32h7xx_hal::stm32::Interrupt::ETH, 196); // Mid prio
        cortex_m::peripheral::NVIC::unmask(stm32h7xx_hal::stm32::Interrupt::ETH);
    }

	eth_dma.number_packets_dropped();

    // Initialise ethernet PHY...
    let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
    lan8742a.phy_reset();
    lan8742a.phy_init();



    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

	writeln!(tx, "Ethernet\r").unwrap();
  


    let mut eth_up = false;
    loop {

		if let Some( pair) = eth_dma.receive() {
			writeln!(tx, "Received!").unwrap();
			//pair.0.consume(timestamp, f)
		}

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
    }
}

#[interrupt]
fn ETH() {
    unsafe { 
		ethernet::interrupt_handler() 
	}
}