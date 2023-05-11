use super::hal::device::SYSCFG;
use super::hal::rcc::Ccdr;
use super::hal::prelude::*;
use super::hal::pac;
use super::hal::rcc::PllConfigStrategy;
use super::hal::pwr::PowerConfiguration;

pub fn init_power(pwr: pac::PWR, system_config: &SYSCFG) -> PowerConfiguration {
	let pwr = pwr.constrain();
	pwr.vos0(system_config).freeze()
}

pub fn init_clock(reset_and_clock_control: pac::RCC, power_config: PowerConfiguration, system_config: &SYSCFG) -> Ccdr {

	let rcc = reset_and_clock_control.constrain();

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
		.freeze(power_config, system_config);

	ccdr
}