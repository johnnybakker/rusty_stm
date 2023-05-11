#![no_std]
#![no_main]

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {

	use core::panic::PanicInfo;
	use core::sync::atomic::{self, Ordering};
	use rusty_stm32::prelude::*;
	use rusty_stm32::hal::gpio::gpioc::PC13;
	use rusty_stm32::hal::gpio::{Output, PushPull};
	use rusty_stm32::hal::stm32::TIM1;
	use rusty_stm32::hal::time::MilliSeconds;
	use rusty_stm32::hal::timer::{Event, Timer};

    #[shared]
    struct SharedResources {}

    #[local]
    struct LocalResources {
		timer1: Timer<TIM1>,
        led: PC13<Output<PushPull>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {	
		
		// Initialize power configuration
		let power_config = init_power(ctx.device.PWR, &ctx.device.SYSCFG);

		// Initialize clock frequencies and configuration
		let core_clock = init_clock(ctx.device.RCC, power_config, &ctx.device.SYSCFG);

		let mut timer1 = ctx.device.TIM1.timer(
			MilliSeconds::millis(1000).into_rate(),
			core_clock.peripheral.TIM1,
			&core_clock.clocks,
		);

		timer1.listen(Event::TimeOut);

		let gpioc = ctx.device.GPIOC.split(core_clock.peripheral.GPIOC);	
     

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

	#[inline(never)]
	#[panic_handler]
	fn panic(_info: &PanicInfo) -> ! {
		loop {
			atomic::compiler_fence(Ordering::SeqCst);
		}
	}
}

