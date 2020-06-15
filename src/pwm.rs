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

pub trait Pins<TIM, CHS> {
    type Channels;
    type PwmController;
    fn setup(&self);
    fn reset(&self);
}

pub trait PwmExt: Sized {
    fn pwm<PINS, CHS, T>(self, _: PINS, frequency: T, rcc: &mut Rcc) -> PINS::Channels
    where
        PINS: Pins<Self, CHS>,
        T: Into<Hertz>;

    fn pwm_controller<PINS, CHS, PWMS, T>(
        self,
        pins: PINS,
        freq: T,
        rcc: &mut Rcc,
    ) -> PINS::PwmController
    where
        PINS: Pins<Self, CHS, Channels = PWMS, PwmController = PwmController<Self, CHS, PWMS, PINS>>,
        T: Into<Hertz>;

    fn pwm_reset<PINS, CHS>(&mut self, pins: &mut PINS, rcc: &mut Rcc)
    where
        PINS: Pins<Self, CHS>;
}

pub struct Pwm<TIM, CHANNEL> {
    _channel: PhantomData<CHANNEL>,
    _tim: PhantomData<TIM>,
}

pub struct PwmController<TIM, CHS, PWMS, PINS>
    where PINS: Pins<TIM, CHS, Channels = PWMS>
{
    tim: TIM,
    _chs: PhantomData<CHS>,
    pwm: PWMS,
    pin: PINS,
}

impl<TIM, CHS, PWMS, PINS> PwmController<TIM, CHS, PWMS, PINS>
    where TIM: PwmExt, PINS: Pins<TIM, CHS, Channels = PWMS>, PWMS: PwmChannelsReset<TIM,CHS,PINS>
{
    pub fn close(mut self, rcc: &mut Rcc) -> (TIM, PINS) {
        self.pwm.reset();
        self.tim.pwm_reset(&mut self.pin, rcc);

        (self.tim, self.pin)
    }
}

macro_rules! trait_pwm_ctrl_chs {
    ( 1 ch ) => {
        pub trait PwmController1Ch<TIM,CH> {
            fn pwm(&mut self) -> &mut Pwm<TIM,CH>;
        }
    };
    ( 2 ch ) => {
        trait_pwm_ctrl_chs!(PwmController2Ch:
            CH1, pwm1,
            CH2, pwm2
        );
    };
    ( 3 ch ) => {
        trait_pwm_ctrl_chs!(PwmController3Ch:
            CH1, pwm1,
            CH2, pwm2,
            CH3, pwm3
        );
    };
    ( 4 ch ) => {
        trait_pwm_ctrl_chs!(PwmController4Ch:
            CH1, pwm1,
            CH2, pwm2,
            CH3, pwm3,
            CH4, pwm4
        );
    };
    ( $nChsTraitName:ident: $($CH:tt, $pwm:ident),+ ) => {
        pub trait $nChsTraitName<TIM, $($CH),+> {$(
            fn $pwm(&mut self) -> &mut Pwm<TIM,$CH>;
        )+}
    }
}
trait_pwm_ctrl_chs!( 1 ch );
trait_pwm_ctrl_chs!( 2 ch );
trait_pwm_ctrl_chs!( 3 ch );
trait_pwm_ctrl_chs!( 4 ch );

pub trait PwmChannelsReset<TIM, CHS, PINS>
    where PINS: Pins<TIM,CHS>
{
    fn reset(&mut self);
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
    ($TIMX:ident, $af:expr, $($c:ident, $pin:ty),+) => {
        $(
            impl Pins<$TIMX, $c> for $pin {
                type Channels = Pwm<$TIMX, $c>;
                type PwmController = PwmController<$TIMX, $c, Pwm<$TIMX, $c>, $pin>;

                fn setup(&self) {
                    self.set_alt_mode($af);
                }
                fn reset(&self) {
                    self.set_alt_mode(AltMode::SYSTEM);
                }
            }
            impl PwmChannelsReset<$TIMX, $c, $pin> for Pwm<$TIMX, $c>
            {
                fn reset(&mut self) {
                    self.disable();
                }
            }
        )+
    };
}

macro_rules! impl_traits_for_tuples {
    ( 2 ch $TIM:ident ) => {
        impl_traits_for_tuples!($TIM: 0, C1, PIN1, 1, C2, PIN2);
    };
    ( 3 ch $TIM:ident ) => {
        impl_traits_for_tuples!( 2 ch $TIM );
        impl_traits_for_tuples!($TIM: 0, C1, PIN1, 1, C3, PIN2);
        impl_traits_for_tuples!($TIM: 0, C2, PIN1, 1, C3, PIN2);
        impl_traits_for_tuples!($TIM: 0, C1, PIN1, 1, C2, PIN2, 2, C3, PIN3 );
    };
    ( 4 ch $TIM:ident ) => {
        impl_traits_for_tuples!( 3 ch $TIM );
        impl_traits_for_tuples!($TIM: 0, C1, PIN1, 1, C4, PIN2);
        impl_traits_for_tuples!($TIM: 0, C2, PIN1, 1, C4, PIN2);
        impl_traits_for_tuples!($TIM: 0, C3, PIN1, 1, C4, PIN2);
        impl_traits_for_tuples!($TIM: 0, C1, PIN1, 1, C2, PIN2, 2, C4, PIN3 );
        impl_traits_for_tuples!($TIM: 0, C1, PIN1, 1, C3, PIN2, 2, C4, PIN3 );
        impl_traits_for_tuples!($TIM: 0, C2, PIN1, 1, C3, PIN2, 2, C4, PIN3 );
        impl_traits_for_tuples!($TIM: 0, C1, PIN1, 1, C2, PIN2, 2, C3, PIN3, 3, C4, PIN4 );
    };
    ( $TIM:ident: $($i:tt, $CH:ident, $PIN:tt),+ ) => {
        impl<$($PIN),+> Pins<$TIM, ($($CH),+)> for ($($PIN),+)
            where $($PIN: Pins<$TIM, $CH>),+
        {
            type Channels = ($(Pwm<$TIM,$CH>,)+);
            type PwmController = PwmController<$TIM, ($($CH),+), Self::Channels, ($($PIN),+)>;

            fn setup(&self) {
                $(
                    self.$i.setup();
                )+
            }

            fn reset(&self) {
                $(
                    self.$i.reset();
                )+
            }
        }
        impl<$($PIN),+> PwmChannelsReset<$TIM, ($($CH),+), ($($PIN),+)>
            for ($(Pwm<$TIM,$CH>),+)
            where $($PIN: Pins<$TIM, $CH>),+
        {
            fn reset(&mut self) {
                $(
                    self.$i.disable();
                )+
            }
        }
    };
}
macro_rules! impl_pwm_ctrl_chs {
    ( 1 ch $TIM:ident ) => {
        impl<PINS,CH> PwmController1Ch<$TIM, CH>
            for PwmController<$TIM, CH, Pwm<$TIM,CH>, PINS>
            where PINS: Pins<$TIM, CH, Channels = Pwm<$TIM,CH>>
        {
            fn pwm(&mut self) -> &mut Pwm<$TIM,CH> {
                &mut self.pwm
            }
        }
    };
    ( 2 ch $TIM:ident ) => {
        impl_pwm_ctrl_chs!( 1 ch $TIM );
        impl_pwm_ctrl_chs!(PwmController2Ch for $TIM:
            pwm1, CH1, 0, pwm2, CH2, 1);
    };
    ( 3 ch $TIM:ident ) => {
        impl_pwm_ctrl_chs!( 2 ch $TIM );
        impl_pwm_ctrl_chs!(PwmController3Ch for $TIM:
            pwm1, CH1, 0, pwm2, CH2, 1, pwm3, CH3, 2);
    };
    ( 4 ch $TIM:ident ) => {
        impl_pwm_ctrl_chs!( 3 ch $TIM );
        impl_pwm_ctrl_chs!(PwmController4Ch for $TIM:
            pwm1, CH1, 0, pwm2, CH2, 1, pwm3, CH3, 2, pwm4, CH4, 3);
    };
    ( $nChsTraitName:ident for $TIM:ident: $($pwm:ident, $CH:tt, $i:tt),+ ) => {
        impl<PINS, $($CH),+> $nChsTraitName<$TIM,$($CH),+>
            for PwmController<$TIM, ($($CH),+), ($(Pwm<$TIM,$CH>),+), PINS>
            where PINS: Pins<$TIM, ($($CH),+), Channels = ($(Pwm<$TIM,$CH>),+)>
        {$(
            fn $pwm(&mut self) -> &mut Pwm<$TIM, $CH> {
                &mut self.pwm.$i
            }
        )+}
    };
}

macro_rules! tim_af{
    ( TIM2 ) => { AltMode::TIM2 };
    ( TIM3 ) => { AltMode::TIM3_5 };
    ( TIM4 ) => { AltMode::TIM3_5 };
    ( TIM5 ) => { AltMode::TIM3_5 };
    ( TIM9 ) => { AltMode::TIM9_11 };
    ( TIM10 ) => { AltMode::TIM9_11 };
    ( TIM11 ) => { AltMode::TIM9_11 };
}

macro_rules! channels {
    ($TIMX:ident, ) => {};
    (init $TIMX:ident: ($c1:ty, $c2:ty, $c3:ty, $c4:ty); $($tail:tt)*) => {
        impl_pins!($TIMX, tim_af!($TIMX), C1, $c1, C2, $c2, C3, $c3, C4, $c4);

        impl_pwm_pin!($TIMX, C1);
        impl_pwm_pin!($TIMX, C2);
        impl_pwm_pin!($TIMX, C3);
        impl_pwm_pin!($TIMX, C4);

        channels!($TIMX, $($tail)*);

        impl_pwm_ctrl_chs!( 4 ch $TIMX );
        impl_traits_for_tuples!( 4 ch $TIMX);
    };
    (init $TIMX:ident: ($c1:ty, $c2:ty); $($tail:tt)*) => {
        impl_pins!($TIMX, tim_af!($TIMX), C1, $c1, C2, $c2);

        impl_pwm_pin!($TIMX, C1);
        impl_pwm_pin!($TIMX, C2);

        channels!($TIMX, $($tail)*);

        impl_pwm_ctrl_chs!( 2 ch $TIMX );
        impl_traits_for_tuples!( 2 ch $TIMX );
    };
    (init $TIMX:ident:($c1:ty); $($tail:tt)*) => {
        impl_pins!($TIMX, tim_af!($TIMX), C1, $c1);

        impl_pwm_pin!($TIMX, C1);

        channels!($TIMX, $($tail)*);

        impl_pwm_ctrl_chs!( 1 ch $TIMX );
    };
    ($TIMX:ident, $($c:ident, $pin:ty),+) => {
        impl_pins!($TIMX, tim_af!($TIMX), $($c, $pin),+);
    };
}

macro_rules! timers {
    ($($TIMX:ident: ($apb_clk:ident, $apbXenr:ident, $apbXrstr:ident, $timX:ident, $timXen:ident, $timXrst:ident),)+) => {
        $(
            impl PwmExt for $TIMX {
                fn pwm<PINS, CHS, T>(
                    self,
                    pins: PINS,
                    freq: T,
                    rcc: &mut Rcc,
                ) -> PINS::Channels
                where
                    PINS: Pins<Self, CHS>,
                    T: Into<Hertz>,
                {
                    let (_tim, pwm, _pins) = $timX(self, pins, freq.into(), rcc);
                    pwm
                }
                fn pwm_controller<PINS, CHS, PWMS, T>(
                    self,
                    pins: PINS,
                    freq: T,
                    rcc: &mut Rcc,
                ) -> PINS::PwmController
                where
                    PINS: Pins<Self, CHS, Channels = PWMS, PwmController = PwmController<Self, CHS, PWMS, PINS>>,
                    T: Into<Hertz>,
                {
                    let (tim, pwm, pin) = $timX(self, pins, freq.into(), rcc);
                    let pwm_ctrl = PwmController { tim, pwm, pin, _chs: PhantomData {} };
                    pwm_ctrl
                }

                fn pwm_reset<PINS,CHS>(&mut self, pins: &mut PINS, rcc: &mut Rcc)
                where
                    PINS: Pins<Self,CHS>,
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

            fn $timX<PINS, CHS>(
                tim: $TIMX,
                pins: PINS,
                freq: Hertz,
                rcc: &mut Rcc,
            ) -> ($TIMX, PINS::Channels, PINS)
            where
                PINS: Pins<$TIMX, CHS>,
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

channels!(init TIM2: (
        PA0<Input<Floating>>,
        PA1<Input<Floating>>,
        PA2<Input<Floating>>,
        PA3<Input<Floating>>
    );

    C1, PA5<Input<Floating>>,

    C3, PB10<Input<Floating>>,
    C4, PB11<Input<Floating>>,

    C1, PA15<Input<Floating>>,

    C2, PB3<Input<Floating>>
);
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
channels!(TIM2,
    C1, PE9<Input<Floating>>,
    C2, PE10<Input<Floating>>,
    C3, PE11<Input<Floating>>,
    C4, PE12<Input<Floating>>
);

channels!(init TIM3: (
        PA6<Input<Floating>>,
        PA7<Input<Floating>>,
        PB0<Input<Floating>>,
        PB1<Input<Floating>>
    );

    C1, PC6<Input<Floating>>,
    C2, PC7<Input<Floating>>,
    C3, PC8<Input<Floating>>,
    C4, PC9<Input<Floating>>,

    C1, PB4<Input<Floating>>,
    C2, PB5<Input<Floating>>
);
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
channels!(TIM3,
    C1, PE3<Input<Floating>>,
    C2, PE4<Input<Floating>>
);

channels!(init TIM4: (
        PB6<Input<Floating>>,
        PB7<Input<Floating>>,
        PB8<Input<Floating>>,
        PB9<Input<Floating>>
    );

    C1, PD12<Input<Floating>>,
    C2, PD13<Input<Floating>>,
    C3, PD14<Input<Floating>>,
    C4, PD15<Input<Floating>>
);

channels!(init TIM5: (
    PA0<Input<Floating>>,
    PA1<Input<Floating>>,
    PA2<Input<Floating>>,
    PA3<Input<Floating>>
););
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
channels!(TIM5,
    C1, PF6<Input<Floating>>,
    C2, PF7<Input<Floating>>,
    C3, PF8<Input<Floating>>,
    C4, PF9<Input<Floating>>
);

channels!(init TIM9: (
        PA2<Input<Floating>>,
        PA3<Input<Floating>>
    );

    C1, PB13<Input<Floating>>,
    C2, PB14<Input<Floating>>,

    C1, PD0<Input<Floating>>,

    C2, PD7<Input<Floating>>
);
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
channels!(TIM9,
    C1, PE5<Input<Floating>>,
    C2, PE6<Input<Floating>>
);

channels!(init TIM10: (
        PA6<Input<Floating>>
    );

    C1, PB8<Input<Floating>>,
    C1, PB12<Input<Floating>>
);
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
channels!(TIM10,
    C1, PE0<Input<Floating>>
);

channels!(init TIM11: (
        PA7<Input<Floating>>
    );

    C1, PB9<Input<Floating>>,
    C1, PB15<Input<Floating>>
);
#[cfg(any(feature = "stm32l151", feature = "stm32l152", feature = "stm32l162"))]
channels!(TIM11,
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
