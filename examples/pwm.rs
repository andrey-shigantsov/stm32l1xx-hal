#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_semihosting;
extern crate stm32l1xx_hal as hal;

use cortex_m::asm;
use hal::prelude::*;
use hal::rcc::Config;
use hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let dph = stm32::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(Config::hsi());

    let mut gpioa = dph.GPIOA.split(&mut rcc);
    let mut gpiob = dph.GPIOB.split(&mut rcc);
    if true { /* Simple tuple */
        let mut (pwm1, pwm2) = dph.TIM9.pwm(
            (gpioa.pa2, gpioa.pa3), 10.khz(), &mut rcc);

        do_pwm_test(&mut pwm1);
        do_pwm_test(&mut pwm2);
    } else { /* Simple */
        let mut pwm1 = dph.TIM9.pwm(
            (gpioa.pa2, gpioa.pa3), 10.khz(), &mut rcc);

        do_pwm_test(&mut pwm1);
    }
    /* CONTROLLER */ {
        /* TIM2 (CH1, CH2) */ {
            let mut tim2pwmc = dph.TIM2.pwm_controller(
                (gpioa.pa0,gpioa.pa1), 10.khz(), &mut rcc);

            do_pwm_test(tim2pwmc.pwm1());
            do_pwm_test(tim2pwmc.pwm2());

            let (tim, (pa0, pa1)) = tim2pwmc.close(&mut rcc);
            dph.TIM2 = tim; gpioa.pa0 = pa0; gpioa.pa1 = pa1;
        }
        /* TIM3 (CH1, CH3) */ {
            let mut tim3pwmc = dph.TIM3.pwm_controller(
                (gpioa.pa6, gpiob.pb0), 10.khz(), &mut rcc);

            do_pwm_test(tim3pwmc.pwm1());
            do_pwm_test(tim3pwmc.pwm2());

            let (tim, (pa6, pb0)) = tim3pwmc.close(&mut rcc);
            dph.TIM3 = tim; gpiob.pb0 = pb0; gpioa.pa6 = pa6;
        }
        /* TIM3 (CH2, CH4) */ {
            let mut tim3pwmc = dph.TIM3.pwm_controller(
                (gpioa.pa7, gpiob.pb1), 10.khz(), &mut rcc);

            do_pwm_test(tim3pwmc.pwm1());
            do_pwm_test(tim3pwmc.pwm2());

            let (tim, (pa7, pb1)) = tim3pwmc.close(&mut rcc);
            dph.TIM3 = tim; gpiob.pb1 = pb1; gpioa.pa7 = pa7;
        }
    };

    fn do_pwm_test<T>(pwm: &mut T)
        where T: hal::hal::PwmPin<Duty = u16>
    {
        let max = pwm.get_max_duty();
        pwm.enable();
        for i in 1..=4 {
            pwm.set_duty(max * i*2/10);
            for _j in 1..10 {
                asm::nop();
            }
        }
        pwm.disable();
    }

    loop {}
}
