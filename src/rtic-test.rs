#![no_std]
#![no_main]

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {
    use stm32h7xx_hal::gpio::gpioc::{PC13};
    use stm32h7xx_hal::gpio::{Edge, ExtiPin, Input};
    use stm32h7xx_hal::gpio::{Output, PushPull};
    use stm32h7xx_hal::pac;
	use stm32h7xx_hal::stm32::{TIM1, TIM12, TIM17, TIM2};
    use stm32h7xx_hal::rcc::PllConfigStrategy;
	use stm32h7xx_hal::time::MilliSeconds;
    use stm32h7xx_hal::timer::{Event, Timer};
    use stm32h7xx_hal::prelude::*;
	use panic_halt as _;


    #[shared]
    struct SharedResources {}

    #[local]
    struct LocalResources {
		timer1: Timer<TIM1>,
        led: PC13<Output<PushPull>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {	
		// Constrain and Freeze power
		let pwrcfg = ctx.device.PWR.constrain().vos0(&ctx.device.SYSCFG).freeze();
		
		// Constrain and Freeze clock
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

		let mut timer1 = ctx.device.TIM1.timer(
			MilliSeconds::millis(1000).into_rate(),
			ccdr.peripheral.TIM1,
			&ccdr.clocks,
		);
		timer1.listen(Event::TimeOut);

		let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);	
     

        (
            SharedResources {},
            LocalResources {
                timer1,
                led: gpioc.pc13.into_push_pull_output(),
            },
            init::Monotonics(),
        )
    }

    #[task(binds = TIM1_UP, local = [led, timer1])]
    fn timer1_tick(ctx: timer1_tick::Context) {
        ctx.local.timer1.clear_irq();
        ctx.local.led.toggle();
    }
}