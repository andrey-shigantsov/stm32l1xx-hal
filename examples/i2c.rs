#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate panic_semihosting;
extern crate stm32l1xx_hal as hal;

use hal::prelude::*;
use hal::stm32;
use rt::entry;
use sh::hprintln;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpiob = dp.GPIOB.split();

    let scl = gpiob.pb10.into_alternate_af4();
    let sda = gpiob.pb11.into_alternate_af4();

    let mut i2c = dp.I2C2.i2c((scl, sda), 10.khz(), clocks);

    let mut buf: [u8; 1] = [0; 1];

    loop {
        match i2c.read(0x60, &mut buf) {
            Ok(_) => hprintln!("Buf: {:?}", buf).unwrap(),
            Err(err) => hprintln!("Err: {:?}", err).unwrap(),
        }
    }
}
