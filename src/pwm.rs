use core::marker::PhantomData;
use core::mem;

use crate::gpio::gpioa::{PA0, PA1, PA2, PA3, PA5, PA6, PA7, PA15};
use crate::gpio::gpiob::{PB0, PB1, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15};
use crate::gpio::gpioc::{PC6, PC7, PC8, PC9};
use crate::gpio::gpiod::{PD0, PD7, PD12, PD13, PD14, PD15};
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
use crate::gpio::gpioe::{PE0, PE1, PE3, PE4, PE5, PE6, PE9, PE10, PE11, PE12};
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
use crate::gpio::gpiof::{PF6, PF7, PF8, PF9};
use crate::gpio::{AltMode, Floating, Input};
use crate::rcc::Rcc;
use crate::stm32::{TIM10, TIM11, TIM2, TIM3, TIM4, TIM5, TIM9};
use crate::time::Hertz;
use cast::{u16, u32};
use hal::{self, PwmPin};

pub struct C1;
pub struct C2;
pub struct C3;
pub struct C4;

pub trait Pins<TIM> {
    type Channels;
    type PwmController;
    fn setup(&self);
    fn reset(&self);
}

pub trait PwmExt: Sized {
    fn pwm<PINS, T>(self, _: PINS, frequency: T, rcc: &mut Rcc) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>;

    fn pwm_controller<PINS, CHS, T>(
        self,
        pins: PINS,
        freq: T,
        rcc: &mut Rcc,
    ) -> PINS::PwmController
    where
        PINS: Pins<Self, Channels = CHS, PwmController = PwmController<Self, CHS, PINS>>,
        T: Into<Hertz>;

    fn pwm_reset<PINS>(&mut self, pins: &mut PINS, rcc: &mut Rcc)
    where
        PINS: Pins<Self>;
}

pub struct Pwm<TIM, CHANNEL> {
    _channel: PhantomData<CHANNEL>,
    _tim: PhantomData<TIM>,
}

pub struct PwmController<TIM, PWM_CHANNELS, PINS>
    where PINS: Pins<TIM, Channels = PWM_CHANNELS>
{
    tim: TIM,
    pwm: PWM_CHANNELS,
    pin: PINS,
}

pub trait PwmController1Ch<TIM, CH>
{
    fn pwm(&mut self) -> &mut Pwm<TIM, CH>;
}
pub trait PwmController2Chs<TIM, CH1, CH2> {
    fn pwm1(&mut self) -> &mut Pwm<TIM, CH1>;
    fn pwm2(&mut self) -> &mut Pwm<TIM, CH2>;
}
pub trait PwmController3Chs<TIM, CH1, CH2, CH3> {
    fn pwm1(&mut self) -> &mut Pwm<TIM, CH1>;
    fn pwm2(&mut self) -> &mut Pwm<TIM, CH2>;
    fn pwm3(&mut self) -> &mut Pwm<TIM, CH3>;
}
pub trait PwmController4Chs<TIM, CH1, CH2, CH3, CH4> {
    fn pwm1(&mut self) -> &mut Pwm<TIM, CH1>;
    fn pwm2(&mut self) -> &mut Pwm<TIM, CH2>;
    fn pwm3(&mut self) -> &mut Pwm<TIM, CH3>;
    fn pwm4(&mut self) -> &mut Pwm<TIM, CH4>;
}

impl<TIM, PWMS, PINS> PwmController<TIM, PWMS, PINS>
    where TIM: PwmExt, PINS: Pins<TIM, Channels = PWMS>, PWMS: PwmChannelsReset<PINS>
{
    pub fn close(mut self, rcc: &mut Rcc) -> (TIM, PINS) {
        self.pwm.reset();
        self.tim.pwm_reset(&mut self.pin, rcc);

        (self.tim, self.pin)
    }
}

pub trait PwmChannelsReset<PINS> {
    fn reset(&mut self);
}

impl<TIM, PIN, CH> PwmController1Ch<TIM, CH> for PwmController<TIM, Pwm<TIM, CH>, PIN>
    where PIN: Pins<TIM, Channels = Pwm<TIM, CH>>
{
    fn pwm(&mut self) -> &mut Pwm<TIM, CH> {
        &mut self.pwm
    }
}

impl<TIM, PIN, CH1, CH2> PwmController2Chs<TIM, CH1, CH2> for PwmController<TIM, (Pwm<TIM, CH1>, Pwm<TIM, CH2>), PIN>
    where PIN: Pins<TIM, Channels = (Pwm<TIM, CH1>,Pwm<TIM, CH2>)>
{
    fn pwm1(&mut self) -> &mut Pwm<TIM, CH1> {
        &mut self.pwm.0
    }
    fn pwm2(&mut self) -> &mut Pwm<TIM, CH2> {
        &mut self.pwm.1
    }
}

impl<TIM, PIN, CH1, CH2, CH3> PwmController3Chs<TIM, CH1, CH2, CH3> for PwmController<TIM, (Pwm<TIM, CH1>, Pwm<TIM, CH2>, Pwm<TIM, CH3>), PIN>
    where PIN: Pins<TIM, Channels = (Pwm<TIM, CH1>, Pwm<TIM, CH2>, Pwm<TIM, CH3>)>
{
    fn pwm1(&mut self) -> &mut Pwm<TIM, CH1> {
        &mut self.pwm.0
    }
    fn pwm2(&mut self) -> &mut Pwm<TIM, CH2> {
        &mut self.pwm.1
    }
    fn pwm3(&mut self) -> &mut Pwm<TIM, CH3> {
        &mut self.pwm.2
    }
}

impl<TIM, PIN, CH1, CH2, CH3, CH4> PwmController4Chs<TIM, CH1, CH2, CH3, CH4> for PwmController<TIM, (Pwm<TIM, CH1>, Pwm<TIM, CH2>, Pwm<TIM, CH3>, Pwm<TIM, CH4>), PIN>
    where PIN: Pins<TIM, Channels = (Pwm<TIM, CH1>, Pwm<TIM, CH2>, Pwm<TIM, CH3>, Pwm<TIM, CH4>)>
{
    fn pwm1(&mut self) -> &mut Pwm<TIM, CH1> {
        &mut self.pwm.0
    }
    fn pwm2(&mut self) -> &mut Pwm<TIM, CH2> {
        &mut self.pwm.1
    }
    fn pwm3(&mut self) -> &mut Pwm<TIM, CH3> {
        &mut self.pwm.2
    }
    fn pwm4(&mut self) -> &mut Pwm<TIM, CH4> {
        &mut self.pwm.3
    }
}

macro_rules! impl_pwm_pin {
    (TIM9, C1) => {
        impl hal::PwmPin for Pwm<TIM9, C1> {
            type Duty = u16;

            fn disable(&mut self) {
                // unsafe {
                //     (*TIM9::ptr()).ccer.modify(|_, w| w.cc1e().clear_bit());
                // }
            }

            fn enable(&mut self) {
                unsafe {
                    let tim = &*TIM9::ptr();
                    tim.ccmr1_output()
                        .modify(|_, w| w.oc1pe().set_bit().oc1m().bits(6));
                    // tim.ccer.modify(|_, w| w.cc1e().set_bit());
                }
            }

            fn get_duty(&self) -> u16 {
                unsafe { (*TIM9::ptr()).ccr1.read().ccr().bits() }
            }

            fn get_max_duty(&self) -> u16 {
                unsafe { (*TIM9::ptr()).arr.read().arr().bits() }
            }

            fn set_duty(&mut self, duty: u16) {
                unsafe { (*TIM9::ptr()).ccr1.write(|w| w.ccr().bits(duty)) }
            }
        }
    };
    (TIM9, C2) => {
        impl hal::PwmPin for Pwm<TIM9, C2> {
            type Duty = u16;

            fn disable(&mut self) {
                // unsafe {
                //     (*TIM9::ptr()).ccer.modify(|_, w| w.cc2e().clear_bit());
                // }
            }

            fn enable(&mut self) {
                unsafe {
                    let tim = &*TIM9::ptr();
                    tim.ccmr1_output()
                        .modify(|_, w| w.oc2pe().set_bit().oc2m().bits(6));
                    // tim.ccer.modify(|_, w| w.cc2e().set_bit());
                }
            }

            fn get_duty(&self) -> u16 {
                unsafe { (*TIM9::ptr()).ccr2.read().ccr().bits() }
            }

            fn get_max_duty(&self) -> u16 {
                unsafe { (*TIM9::ptr()).arr.read().arr().bits() }
            }

            fn set_duty(&mut self, duty: u16) {
                unsafe { (*TIM9::ptr()).ccr2.write(|w| w.ccr().bits(duty)) }
            }
        }
    };
    ($TIMX:ident, C1) => {
        impl hal::PwmPin for Pwm<$TIMX, C1> {
            type Duty = u16;

            fn disable(&mut self) {
                unsafe {
                    (*$TIMX::ptr()).ccer.modify(|_, w| w.cc1e().clear_bit());
                }
            }

            fn enable(&mut self) {
                unsafe {
                    let tim = &*$TIMX::ptr();
                    tim.ccmr1_output()
                        .modify(|_, w| w.oc1pe().set_bit().oc1m().bits(6));
                    tim.ccer.modify(|_, w| w.cc1e().set_bit());
                }
            }

            fn get_duty(&self) -> u16 {
                unsafe { (*$TIMX::ptr()).ccr1.read().ccr().bits() }
            }

            fn get_max_duty(&self) -> u16 {
                unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
            }

            fn set_duty(&mut self, duty: u16) {
                unsafe { (*$TIMX::ptr()).ccr1.write(|w| w.ccr().bits(duty)) }
            }
        }
    };
    ($TIMX:ident, C2) => {
        impl hal::PwmPin for Pwm<$TIMX, C2> {
            type Duty = u16;

            fn disable(&mut self) {
                unsafe {
                    (*$TIMX::ptr()).ccer.modify(|_, w| w.cc2e().clear_bit());
                }
            }

            fn enable(&mut self) {
                unsafe {
                    let tim = &*$TIMX::ptr();
                    tim.ccmr1_output()
                        .modify(|_, w| w.oc2pe().set_bit().oc2m().bits(6));
                    tim.ccer.modify(|_, w| w.cc2e().set_bit());
                }
            }

            fn get_duty(&self) -> u16 {
                unsafe { (*$TIMX::ptr()).ccr2.read().ccr().bits() }
            }

            fn get_max_duty(&self) -> u16 {
                unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
            }

            fn set_duty(&mut self, duty: u16) {
                unsafe { (*$TIMX::ptr()).ccr2.write(|w| w.ccr().bits(duty)) }
            }
        }
    };
    ($TIMX:ident, C3) => {
        impl hal::PwmPin for Pwm<$TIMX, C3> {
            type Duty = u16;

            fn disable(&mut self) {
                unsafe {
                    (*$TIMX::ptr()).ccer.modify(|_, w| w.cc3e().clear_bit());
                }
            }

            fn enable(&mut self) {
                unsafe {
                    let tim = &*$TIMX::ptr();
                    tim.ccmr2_output()
                        .modify(|_, w| w.oc3pe().set_bit().oc3m().bits(6));
                    tim.ccer.modify(|_, w| w.cc3e().set_bit());
                }
            }

            fn get_duty(&self) -> u16 {
                unsafe { (*$TIMX::ptr()).ccr3.read().ccr().bits() }
            }

            fn get_max_duty(&self) -> u16 {
                unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
            }

            fn set_duty(&mut self, duty: u16) {
                unsafe { (*$TIMX::ptr()).ccr3.write(|w| w.ccr().bits(duty)) }
            }
        }
    };
    ($TIMX:ident, C4) => {
        impl hal::PwmPin for Pwm<$TIMX, C4> {
            type Duty = u16;

            fn disable(&mut self) {
                unsafe {
                    (*$TIMX::ptr()).ccer.modify(|_, w| w.cc4e().clear_bit());
                }
            }

            fn enable(&mut self) {
                unsafe {
                    let tim = &*$TIMX::ptr();
                    tim.ccmr2_output()
                        .modify(|_, w| w.oc4pe().set_bit().oc4m().bits(6));
                    tim.ccer.modify(|_, w| w.cc4e().set_bit());
                }
            }

            fn get_duty(&self) -> u16 {
                unsafe { (*$TIMX::ptr()).ccr4.read().ccr().bits() }
            }

            fn get_max_duty(&self) -> u16 {
                unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
            }

            fn set_duty(&mut self, duty: u16) {
                unsafe { (*$TIMX::ptr()).ccr4.write(|w| w.ccr().bits(duty)) }
            }
        }
    };

}

macro_rules! impl_pins {
     ($TIMX:ident, $af:expr, ($CX:ident: $pinx:ty, $CY:ident: $piny:ty)) => {
        impl Pins<$TIMX> for ($pinx, $piny) {
            type Channels = (
                Pwm<$TIMX, $CX>,
                Pwm<$TIMX, $CY>
            );
            type PwmController = PwmController<$TIMX, (Pwm<$TIMX, $CX>, Pwm<$TIMX, $CY>), ($pinx, $piny)>;

            fn setup(&self) {
                self.0.set_alt_mode($af);
                self.1.set_alt_mode($af);
            }

            fn reset(&self) {
                self.0.set_alt_mode(AltMode::SYSTEM);
                self.1.set_alt_mode(AltMode::SYSTEM);
            }
        }
        impl PwmChannelsReset<($pinx, $piny)> for (Pwm<$TIMX, $CX>, Pwm<$TIMX, $CY>)
        {
            fn reset(&mut self) {
                self.0.disable();
                self.1.disable();
            }
        }

    };
    ($TIMX:ident, $af:expr, ($CX:ident: $pinx:ty, $CY:ident: $piny:ty, $CZ:ident: $pinz:ty)) => {
        impl Pins<$TIMX> for ($pinx, $piny, $pinz) {
            type Channels = (
                Pwm<$TIMX, $CX>,
                Pwm<$TIMX, $CY>,
                Pwm<$TIMX, $CZ>,
            );
            type PwmController = PwmController<$TIMX, (Pwm<$TIMX, $CX>, Pwm<$TIMX, $CY>, , Pwm<$TIMX, $CZ>), ($pinx, $piny, $pinz)>;

            fn setup(&self) {
                self.0.set_alt_mode($af);
                self.1.set_alt_mode($af);
                self.2.set_alt_mode($af);
            }

            fn reset(&self) {
                self.0.set_alt_mode(AltMode::SYSTEM);
                self.1.set_alt_mode(AltMode::SYSTEM);
                self.2.set_alt_mode(AltMode::SYSTEM);
            }
        }
        impl PwmChannelsReset<($pinx, $piny, $pinz)> for (Pwm<$TIMX, $CX>, Pwm<$TIMX, $CY>, Pwm<$TIMX, $CZ>)
        {
            fn reset(&mut self) {
                self.0.disable();
                self.1.disable();
                self.2.disable();
            }
        }

    };
    ($TIMX:ident, $af:expr, $($c:ident, $pin:ty),+) => {
        $(
            impl Pins<$TIMX> for $pin {
                type Channels = Pwm<$TIMX, $c>;
                type PwmController = PwmController<$TIMX, Pwm<$TIMX, $c>, $pin>;

                fn setup(&self) {
                    self.set_alt_mode($af);
                }
                fn reset(&self) {
                    self.set_alt_mode(AltMode::SYSTEM);
                }
            }
            impl PwmChannelsReset<$pin> for Pwm<$TIMX, $c>
            {
                fn reset(&mut self) {
                    self.disable();
                }
            }
        )+
    };
}

macro_rules! channels {
    ($TIMX:ident, $af:expr, $($c:ident, $pin:ty),+) => {
        impl_pins!($TIMX, $af, $($c, $pin),+);
    };
    ($TIMX:ident, $af:expr, $c1:ty) => {
        impl_pins!($TIMX, $af, C1, $c1);
        impl_pwm_pin!($TIMX, C1);
    };
    ($TIMX:ident, $af:expr, $c1:ty, $c2:ty) => {
        channels!($TIMX, $af, $c1);

        impl_pins!($TIMX, $af, C2, $c2);
        impl_pins!($TIMX, $af, (C1: $c1, C2: $c2));

        impl_pwm_pin!($TIMX, C2);
    };
    ($TIMX:ident, $af:expr, $c1:ty, $c2:ty, $c3:ty, $c4:ty) => {
        channels!($TIMX, $af, $c1, $c2);

        impl_pins!($TIMX, $af, C3, $c3, C4, $c4);
        impl_pins!($TIMX, $af, (C1: $c1, C3: $c3));
        impl_pins!($TIMX, $af, (C1: $c1, C4: $c4));
        impl_pins!($TIMX, $af, (C2: $c2, C3: $c3));
        impl_pins!($TIMX, $af, (C2: $c2, C4: $c4));
        impl_pins!($TIMX, $af, (C3: $c3, C4: $c4));
        impl Pins<$TIMX> for ($c1, $c2, $c3, $c4) {
            type Channels = (
                Pwm<$TIMX, C1>,
                Pwm<$TIMX, C2>,
                Pwm<$TIMX, C3>,
                Pwm<$TIMX, C4>,
            );
            type PwmController = PwmController<$TIMX, (Pwm<$TIMX, C1>, Pwm<$TIMX, C2>, Pwm<$TIMX, C3>, Pwm<$TIMX, C4>), ($c1, $c2, $c3, $c4)>;

            fn setup(&self) {
                self.0.set_alt_mode($af);
                self.1.set_alt_mode($af);
                self.2.set_alt_mode($af);
                self.3.set_alt_mode($af);
            }

            fn reset(&self) {
                self.0.set_alt_mode(AltMode::SYSTEM);
                self.1.set_alt_mode(AltMode::SYSTEM);
                self.2.set_alt_mode(AltMode::SYSTEM);
                self.3.set_alt_mode(AltMode::SYSTEM);
            }
        }
        impl PwmChannelsReset<($c1, $c2, $c3, $c4)> for (Pwm<$TIMX, C1>, Pwm<$TIMX, C2>, Pwm<$TIMX, C3>, Pwm<$TIMX, C4>)
        {
            fn reset(&mut self) {
                self.0.disable();
                self.1.disable();
                self.2.disable();
                self.3.disable();
            }
        }

        impl_pwm_pin!($TIMX, C3);
        impl_pwm_pin!($TIMX, C4);
    };
}

macro_rules! timers {
    ($($TIMX:ident: ($apb_clk:ident, $apbXenr:ident, $apbXrstr:ident, $timX:ident, $timXen:ident, $timXrst:ident),)+) => {
        $(
            impl PwmExt for $TIMX {
                fn pwm<PINS, T>(
                    self,
                    pins: PINS,
                    freq: T,
                    rcc: &mut Rcc,
                ) -> PINS::Channels
                where
                    PINS: Pins<Self>,
                    T: Into<Hertz>,
                {
                    let (_tim, pwm, _pins) = $timX(self, pins, freq.into(), rcc);
                    pwm
                }
                fn pwm_controller<PINS, CHS, T>(
                    self,
                    pins: PINS,
                    freq: T,
                    rcc: &mut Rcc,
                ) -> PINS::PwmController
                where
                    PINS: Pins<Self, Channels = CHS, PwmController = PwmController<Self, CHS, PINS>>,
                    T: Into<Hertz>,
                {
                    let (tim, pwm, pin) = $timX(self, pins, freq.into(), rcc);
                    let pwm_ctrl = PwmController { tim, pwm, pin };
                    pwm_ctrl
                }

                fn pwm_reset<PINS>(&mut self, pins: &mut PINS, rcc: &mut Rcc)
                where
                    PINS: Pins<Self>,
                {
                    self.psc.write(|w| w.psc().bits(0) );
                    #[allow(unused_unsafe)]
                    self.arr.write(|w| unsafe { w.arr().bits(0) });
                    self.cr1.write(|w| w.cen().clear_bit());

                    rcc.rb.$apbXenr.modify(|_, w| w.$timXen().clear_bit());
                    rcc.rb.$apbXrstr.modify(|_, w| w.$timXrst().set_bit());
                    rcc.rb.$apbXrstr.modify(|_, w| w.$timXrst().clear_bit());
                    pins.reset();
                }
            }

            fn $timX<PINS>(
                tim: $TIMX,
                pins: PINS,
                freq: Hertz,
                rcc: &mut Rcc,
            ) -> ($TIMX, PINS::Channels, PINS)
            where
                PINS: Pins<$TIMX>,
            {
                pins.setup();
                rcc.rb.$apbXenr.modify(|_, w| w.$timXen().set_bit());
                rcc.rb.$apbXrstr.modify(|_, w| w.$timXrst().set_bit());
                rcc.rb.$apbXrstr.modify(|_, w| w.$timXrst().clear_bit());

                let clk = rcc.clocks.$apb_clk().0;
                let freq = freq.0;
                let ticks = clk / freq;
                let psc = u16((ticks - 1) / (1 << 16)).unwrap();
                let arr = u16(ticks / u32(psc + 1)).unwrap();
                tim.psc.write(|w| w.psc().bits(psc) );
                #[allow(unused_unsafe)]
                tim.arr.write(|w| unsafe { w.arr().bits(arr) });
                tim.cr1.write(|w| w.cen().set_bit());

                (tim, unsafe { mem::MaybeUninit::uninit().assume_init() }, pins)
            }
        )+
    }
}

channels!(
    TIM2,
    AltMode::TIM2,
    PA0<Input<Floating>>,
    PA1<Input<Floating>>,
    PA2<Input<Floating>>,
    PA3<Input<Floating>>
);
channels!(TIM2, AltMode::TIM2,
    C1, PA5<Input<Floating>>,

    C3, PB10<Input<Floating>>,
    C4, PB11<Input<Floating>>,

    C1, PA15<Input<Floating>>,

    C2, PB3<Input<Floating>>
);
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
channels!(TIM2, AltMode::TIM2,
    C1, PE9<Input<Floating>>,
    C2, PE10<Input<Floating>>,
    C3, PE11<Input<Floating>>,
    C4, PE12<Input<Floating>>
);

channels!(
    TIM3,
    AltMode::TIM3_5,
    PA6<Input<Floating>>,
    PA7<Input<Floating>>,
    PB0<Input<Floating>>,
    PB1<Input<Floating>>
);
channels!(TIM3, AltMode::TIM3_5,
    C1, PC6<Input<Floating>>,
    C2, PC7<Input<Floating>>,
    C3, PC8<Input<Floating>>,
    C4, PC9<Input<Floating>>,

    C1, PB4<Input<Floating>>,
    C2, PB5<Input<Floating>>
);
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
channels!(TIM3, AltMode::TIM3_5,
    C1, PE3<Input<Floating>>,
    C2, PE4<Input<Floating>>
);

channels!(
    TIM4,
    AltMode::TIM3_5,
    PB6<Input<Floating>>,
    PB7<Input<Floating>>,
    PB8<Input<Floating>>,
    PB9<Input<Floating>>
);
channels!(TIM4, AltMode::TIM3_5,
    C1, PD12<Input<Floating>>,
    C2, PD13<Input<Floating>>,
    C3, PD14<Input<Floating>>,
    C4, PD15<Input<Floating>>
);

channels!(
    TIM5,
    AltMode::TIM3_5,
    PA0<Input<Floating>>,
    PA1<Input<Floating>>,
    PA2<Input<Floating>>,
    PA3<Input<Floating>>
);
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
channels!(TIM5, AltMode::TIM3_5,
    C1, PF6<Input<Floating>>,
    C2, PF7<Input<Floating>>,
    C3, PF8<Input<Floating>>,
    C4, PF9<Input<Floating>>
);

channels!(
    TIM9,
    AltMode::TIM9_11,
    PA2<Input<Floating>>,
    PA3<Input<Floating>>
);
channels!(TIM9,AltMode::TIM9_11,
    C1, PB13<Input<Floating>>,
    C2, PB14<Input<Floating>>,

    C1, PD0<Input<Floating>>,

    C2, PD7<Input<Floating>>
);
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
channels!(TIM9,AltMode::TIM9_11,
    C1, PE5<Input<Floating>>,
    C2, PE6<Input<Floating>>
);

channels!(TIM10, AltMode::TIM9_11, PA6<Input<Floating>>);
channels!(TIM10, AltMode::TIM9_11,
    C1, PB8<Input<Floating>>,
    C1, PB12<Input<Floating>>
);
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
channels!(TIM10, AltMode::TIM9_11,
    C1, PE0<Input<Floating>>
);

channels!(TIM11, AltMode::TIM9_11, PA7<Input<Floating>>);
channels!(TIM11, AltMode::TIM9_11,
    C1, PB9<Input<Floating>>,
    C1, PB15<Input<Floating>>
);
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
channels!(TIM11, AltMode::TIM9_11,
    C1, PE1<Input<Floating>>
);

timers! {
    TIM2: (apb1_clk, apb1enr, apb1rstr, tim2, tim2en, tim2rst),
    TIM3: (apb1_clk, apb1enr, apb1rstr, tim3, tim3en, tim3rst),
    TIM4: (apb1_clk, apb1enr, apb1rstr, tim4, tim4en, tim4rst),
    TIM5: (apb1_clk, apb1enr, apb1rstr, tim5, tim5en, tim5rst),
    TIM9: (apb2_clk, apb2enr, apb2rstr, tim9, tim9en, tim9rst),
    TIM10: (apb2_clk, apb2enr, apb2rstr, tim10, tim10en, tm10rst),
    TIM11: (apb2_clk, apb2enr, apb2rstr, tim11, tim11en, tm11rst),
}
