use stm32::RCC;
use time::{Hertz, U32Ext};

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            cfgr: Config::default(),
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    pub cfgr: Config,
}

/// System clock input mux source
pub enum SysClockSource {
    MSI(MSIRange),
    PLL(PLLSource, PLLMul, PLLDiv),
    HSE(Hertz),
    HSI,
}

/// System clock input mux source
#[derive(Clone, Copy)]
pub enum MSIRange {
    Range0 = 0,
    Range1 = 1,
    Range2 = 2,
    Range3 = 3,
    Range4 = 4,
    Range5 = 5,
    Range6 = 6,
}

impl Default for MSIRange {
    fn default() -> MSIRange {
        MSIRange::Range5
    }
}

/// PLL divider
#[derive(Clone, Copy)]
pub enum PLLDiv {
    Div2 = 1,
    Div3 = 2,
    Div4 = 3,
}

/// PLL multiplier
#[derive(Clone, Copy)]
pub enum PLLMul {
    Mul3 = 0,
    Mul4 = 1,
    Mul6 = 2,
    Mul8 = 3,
    Mul12 = 4,
    Mul16 = 5,
    Mul24 = 6,
    Mul32 = 7,
    Mul48 = 8,
}

/// AHB prescaler
#[derive(Clone, Copy)]
pub enum AHBPrescaler {
    NotDivided = 0,
    Div2 = 0b1000,
    Div4 = 0b1001,
    Div8 = 0b1010,
    Div16 = 0b1011,
    Div64 = 0b1100,
    Div128 = 0b1101,
    Div256 = 0b1110,
    Div512 = 0b1111,
}

/// APB prescaler
#[derive(Clone, Copy)]
pub enum APBPrescaler {
    NotDivided = 0,
    Div2 = 0b100,
    Div4 = 0b101,
    Div8 = 0b110,
    Div16 = 0b111,
}

/// PLL clock input source
#[derive(Clone, Copy)]
pub enum PLLSource {
    HSI,
    HSE(Hertz),
}

/// Clocks configutation
pub struct Config {
    mux: SysClockSource,
    ahb_pre: AHBPrescaler,
    apb1_pre: APBPrescaler,
    apb2_pre: APBPrescaler,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            mux: SysClockSource::MSI(MSIRange::default()),
            ahb_pre: AHBPrescaler::NotDivided,
            apb1_pre: APBPrescaler::NotDivided,
            apb2_pre: APBPrescaler::NotDivided,
        }
    }
}

impl Config {
    pub fn sys_clk_src(mut self, mux: SysClockSource) -> Self {
        self.mux = mux;
        self
    }

    pub fn ahb_pre(mut self, pre: AHBPrescaler) -> Self {
        self.ahb_pre = pre;
        self
    }

    pub fn apb1_pre(mut self, pre: APBPrescaler) -> Self {
        self.apb1_pre = pre;
        self
    }

    pub fn apb2_pre(mut self, pre: APBPrescaler) -> Self {
        self.apb2_pre = pre;
        self
    }

    pub fn freeze(self) -> Clocks {
        let rcc = unsafe { &*RCC::ptr() };
        let (sys_clk, sw_bits) = match self.mux {
            SysClockSource::MSI(range) => {
                let range = range as u8;
                // Set MSI range
                rcc.icscr.write(|w| unsafe { w.msirange().bits(range) });

                // Enable MSI
                rcc.cr.write(|w| w.msion().set_bit());
                while rcc.cr.read().msirdy().bit_is_clear() {}

                let freq = 32_768 * (1 << (range + 1));
                (freq, 0)
            }
            SysClockSource::HSI => {
                // Enable HSI
                rcc.cr.write(|w| w.hsion().set_bit());
                while rcc.cr.read().hsirdy().bit_is_clear() {}

                (16_000_000, 1)
            }
            SysClockSource::HSE(freq) => {
                // Enable HSE
                rcc.cr.write(|w| w.hseon().set_bit());
                while rcc.cr.read().hserdy().bit_is_clear() {}

                (freq.0, 2)
            }
            SysClockSource::PLL(src, mul, div) => {
                let (src_bit, freq) = match src {
                    PLLSource::HSE(freq) => {
                        // Enable HSE
                        rcc.cr.write(|w| w.hseon().set_bit());
                        while rcc.cr.read().hserdy().bit_is_clear() {}
                        (true, freq.0)
                    }
                    PLLSource::HSI => {
                        // Enable HSI
                        rcc.cr.write(|w| w.hsion().set_bit());
                        while rcc.cr.read().hsirdy().bit_is_clear() {}
                        (false, 16_000_000)
                    }
                };

                // Disable PLL
                rcc.cr.write(|w| w.pllon().clear_bit());
                while rcc.cr.read().pllrdy().bit_is_set() {}
                
                let mul_bytes = mul as u8;
                let div_bytes = div as u8;
                
                let freq = match mul {
                    PLLMul::Mul3 => freq * 3,
                    PLLMul::Mul4 => freq * 4,
                    PLLMul::Mul6 => freq * 6,
                    PLLMul::Mul8 => freq * 8,
                    PLLMul::Mul12 => freq * 12,
                    PLLMul::Mul16 => freq * 16,
                    PLLMul::Mul24 => freq * 24,
                    PLLMul::Mul32 => freq * 32,
                    PLLMul::Mul48 => freq * 48,
                };

                let freq = match div {
                    PLLDiv::Div2 => freq / 2,
                    PLLDiv::Div3 => freq / 3,
                    PLLDiv::Div4 => freq / 4,
                };

                rcc.cfgr.write(move |w| unsafe {
                    w.pllmul()
                        .bits(mul_bytes)
                        .plldiv()
                        .bits(div_bytes)
                        .pllsrc()
                        .bit(src_bit)
                });

                // Enable PLL
                rcc.cr.write(|w| w.pllon().set_bit());
                while rcc.cr.read().pllrdy().bit_is_clear() {}

                assert!(freq <= 24.mhz().0);
                (freq, 3)
            }
        };

        rcc.cfgr.modify(|_, w| unsafe {
            w.sw()
                .bits(sw_bits)
                .ppre1()
                .bits(self.apb1_pre as u8)
                .ppre2()
                .bits(self.apb2_pre as u8)
                .hpre()
                .bits(self.ahb_pre as u8)
        });

        let ahb_freq = match self.ahb_pre {
            AHBPrescaler::NotDivided => sys_clk,
            pre => sys_clk / (1 << (pre as u8 + 1)),
        };

        let (apb1_freq, apb1_tim_freq) = match self.apb1_pre {
            APBPrescaler::NotDivided => (sys_clk, sys_clk),
            pre => {
                let freq = sys_clk / (1 << (pre as u8 + 1));
                (freq, sys_clk * 2)
            }
        };

        let (apb2_freq, apb2_tim_freq) = match self.apb2_pre {
            APBPrescaler::NotDivided => (sys_clk, sys_clk),
            pre => {
                let freq = sys_clk / (1 << (pre as u8 + 1));
                (freq, sys_clk * 2)
            }
        };

        Clocks {
            sys_clk: sys_clk.hz(),
            ahb_clk: ahb_freq.hz(),
            apb1_clk: apb1_freq.hz(),
            apb2_clk: apb2_freq.hz(),
            apb1_tim_clk: apb1_tim_freq.hz(),
            apb2_tim_clk: apb2_tim_freq.hz(),
        }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[derive(Clone, Copy)]
pub struct Clocks {
    sys_clk: Hertz,
    ahb_clk: Hertz,
    apb1_clk: Hertz,
    apb1_tim_clk: Hertz,
    apb2_clk: Hertz,
    apb2_tim_clk: Hertz,
}

impl Clocks {
    /// Returns the system (core) frequency
    pub fn sys_clk(&self) -> Hertz {
        self.sys_clk
    }

    /// Returns the frequency of the AHB
    pub fn ahb_clk(&self) -> Hertz {
        self.ahb_clk
    }
    
    /// Returns the frequency of the APB1
    pub fn apb1_clk(&self) -> Hertz {
        self.apb1_clk
    }

    /// Returns the frequency of the APB1 timers
    pub fn apb1_tim_clk(&self) -> Hertz {
        self.apb1_tim_clk
    }

    /// Returns the frequency of the APB2
    pub fn apb2_clk(&self) -> Hertz {
        self.apb2_clk
    }

    /// Returns the frequency of the APB2 timers
    pub fn apb2_tim_clk(&self) -> Hertz {
        self.apb2_tim_clk
    }
}
