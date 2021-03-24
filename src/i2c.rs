//! I2C
use hal::blocking::i2c::{Read, Write, WriteRead};

use crate::gpio::gpiob::{PB6,PB7, PB8,PB9, PB10,PB11};
use crate::gpio::{AltMode, OpenDrain, Output};
use crate::prelude::*;
use crate::rcc::Rcc;
use crate::stm32::{I2C1, I2C2};
use crate::time::Hertz;

/// I2C abstraction
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
    pub timeout: usize,
    event_driven: bool,
    state: State,
    txn: Option<Transaction>,
}

pub trait Pins<I2c> {
    fn setup(&self);
}

macro_rules! i2c_pins {
    ($i2c:ident: $(($scl:ident, $sda:ident)),+) => {$(
        impl Pins<$i2c> for ($scl<Output<OpenDrain>>, $sda<Output<OpenDrain>>) {
            fn setup(&self) {
                self.0.set_alt_mode(AltMode::I2C);
                self.1.set_alt_mode(AltMode::I2C);
            }
        }
    )+}
}

i2c_pins!(I2C1: (PB6,PB7), (PB8,PB9) );
i2c_pins!(I2C2: (PB10,PB11) );

#[derive(Debug, Eq, PartialEq)]
pub enum Error {
    OVERRUN,
    NACK,
    TIMEOUT,
    BUS,
    ARBTRN_LOST,
    
    General,
    WouldBlock,
}

macro_rules! i2c {
    ($I2CX:ident, $i2cx:ident, $i2cxen:ident, $i2crst:ident) => {
        impl<PINS> I2c<$I2CX, PINS> {
            pub fn $i2cx(i2c: $I2CX, pins: PINS, speed: Hertz, rcc: &mut Rcc) -> Self
            where
                PINS: Pins<$I2CX>,
            {
                pins.setup();
                let speed: Hertz = speed.into();

                // Enable clock for I2C
                rcc.rb.apb1enr.modify(|_, w| w.$i2cxen().set_bit());

                // Reset I2C
                rcc.rb.apb1rstr.modify(|_, w| w.$i2crst().set_bit());
                rcc.rb.apb1rstr.modify(|_, w| w.$i2crst().clear_bit());

                // Make sure the I2C unit is disabled so we can configure it
                i2c.cr1.modify(|_, w| w.pe().clear_bit());

                // Calculate settings for I2C speed modes
                let clock = rcc.clocks.apb1_clk().0;
                let freq = clock / 1_000_000;
                assert!(freq >= 2 && freq <= 50);

                // Configure bus frequency into I2C peripheral
                i2c.cr2.write(|w| unsafe { w.freq().bits(freq as u8) });

                let trise = if speed <= 100_u32.khz().into() {
                    freq + 1
                } else {
                    (freq * 300) / 1000 + 1
                };

                // Configure correct rise times
                i2c.trise.write(|w| w.trise().bits(trise as u8));

                // I2C clock control calculation
                if speed <= 100_u32.khz().into() {
                    let ccr = {
                        let ccr = clock / (speed.0 * 2);
                        if ccr < 4 {
                            4
                        } else {
                            ccr
                        }
                    };

                    // Set clock to standard mode with appropriate parameters for selected speed
                    i2c.ccr.write(|w| unsafe {
                        w.f_s()
                            .clear_bit()
                            .duty()
                            .clear_bit()
                            .ccr()
                            .bits(ccr as u16)
                    });
                } else {
                    const DUTYCYCLE: u8 = 0;
                    if DUTYCYCLE == 0 {
                        let ccr = clock / (speed.0 * 3);
                        let ccr = if ccr < 1 { 1 } else { ccr };

                        // Set clock to fast mode with appropriate parameters for selected speed (2:1 duty cycle)
                        i2c.ccr.write(|w| unsafe {
                            w.f_s().set_bit().duty().clear_bit().ccr().bits(ccr as u16)
                        });
                    } else {
                        let ccr = clock / (speed.0 * 25);
                        let ccr = if ccr < 1 { 1 } else { ccr };

                        // Set clock to fast mode with appropriate parameters for selected speed (16:9 duty cycle)
                        i2c.ccr.write(|w| unsafe {
                            w.f_s().set_bit().duty().set_bit().ccr().bits(ccr as u16)
                        });
                    }
                }

                // Enable the I2C processing
                i2c.cr1.modify(|_, w| w.pe().set_bit());

                I2c { 
                    i2c, pins,
                    timeout: 1_000_000,
                    event_driven: false,
                    state: State::Idle,
                    txn: None,
                }
            }

            pub fn event_irq_enable(&self) {
                self.i2c.cr2.modify(|_, w| w.itevten().set_bit());
            }
            pub fn event_irq_disable(&self) {
                self.i2c.cr2.modify(|_, w| w.itevten().clear_bit());
            }
            pub fn buffer_irq_enable(&self) {
                self.i2c.cr2.modify(|_, w| w.itbufen().set_bit());
            }
            pub fn buffer_irq_disable(&self) {
                self.i2c.cr2.modify(|_, w| w.itbufen().clear_bit());
            }
            pub fn error_irq_enable(&self) {
                self.i2c.cr2.modify(|_, w| w.iterren().set_bit());
            }
            pub fn error_irq_disable(&self) {
                self.i2c.cr2.modify(|_, w| w.iterren().clear_bit());
            }

            pub fn release(self) -> ($I2CX, PINS) {
                (self.i2c, self.pins)
            }

            fn sr1_read(&self) -> Result<crate::stm32::i2c1::sr1::R, Error> {
                let sr1 = self.i2c.sr1.read();

                if sr1.af().bit_is_set() {
                    return Err(Error::NACK);
                }
                if sr1.ovr().bit_is_set() {
                    return Err(Error::OVERRUN);
                }
                if sr1.arlo().bit_is_set() {
                    return Err(Error::ARBTRN_LOST);
                }
                if sr1.berr().bit_is_set() {
                    return Err(Error::BUS);
                }
                if sr1.timeout().bit_is_set() {
                    return Err(Error::TIMEOUT);
                }

                Ok(sr1)
            }

            fn wait<F>(&self, fx: F) -> Result<(), Error>
                where F: Fn() -> Result<bool,Error>
            {
                let mut counter = 0_usize;
                let mut is_timeout = false;
                while fx()? {
                    counter += 1;
                    if counter >= self.timeout {
                        is_timeout = true;
                        break;
                    }
                };
                if is_timeout {
                    Err(Error::TIMEOUT)
                } else {
                    Ok(())
                }
            }

            fn sr1_wait<F>(&self, fx: F) -> Result<(), Error>
                where F: Fn(crate::stm32::i2c1::sr1::R) -> bool
            {
                self.wait(|| {
                    let sr1 = self.sr1_read()?;
                    Ok(fx(sr1))
                })
            }

            fn txn_addr(&self) -> u8 {
                if let Some(txn) = self.txn.as_ref() {
                    txn.addr
                } else {
                    0
                }
            }
            fn txn_need_sending(&self) -> bool {
                if let Some(txn) = self.txn.as_ref() {
                    txn.need_sending()
                } else {
                    false
                }
            }
            fn txn_need_receiving(&self) -> bool {
                if let Some(txn) = self.txn.as_ref() {
                    txn.need_receiving()
                } else {
                    false
                }
            }
            fn txn_mode(&self) -> Result<u8/* 0 - tx, 1 - rx */, Error> {
                if let Some(txn) = self.txn.as_ref() {
                    txn.mode().ok_or(Error::General)
                } else {
                    Err(Error::General)
                }
            }
            fn txn_next_byte_for_send(&mut self) -> Result<Option<u8>, Error> {
                if let Some(txn) = self.txn.as_mut() {
                    Ok(txn.next_byte_for_send())
                } else {
                    Err(Error::General)
                }
            }
            fn txn_save_recv_byte(&mut self, byte: u8) -> Result<Option<bool>, Error> {
                if let Some(txn) = self.txn.as_mut() {
                    Ok(txn.save_recv_byte(byte))
                } else {
                    Err(Error::General)
                }
            }
            fn txn_handler(&mut self) -> Result<(), Error> {
                if !(self.txn_need_sending() || self.txn_need_receiving()) {
                    return Err(Error::General);
                }
                               
                if self.txn_need_sending() {
                    self.start_sending();
                    self.polling_common_handler()?;

                    // Send bytes
                    while self.txn_need_sending() {
                        // Wait until we're ready for sending
                        self.sr1_wait(|sr1| sr1.tx_e().bit_is_clear())?;

                        // Push out a byte of data
                        self.event_handler(Event::ReadyForSend)?;

                        // While until byte is transferred
                        self.sr1_wait(|sr1| sr1.btf().bit_is_clear())?;

                        self.event_handler(Event::Sent)?;
                    }
                }

                if self.txn_need_receiving() {
                    self.start_receiving();
                    self.polling_common_handler()?;

                    // Receive bytes into buffer
                    while self.txn_need_receiving() {
                        // Wait until we're ready for receiving
                        self.sr1_wait(|sr1| sr1.rx_ne().bit_is_clear())?;
                        
                        self.event_handler(Event::ReadyForRecv)?;
                    }
                }

                Ok(())
            }

            fn start_sending(&mut self) {
                // Send a START condition
                self.i2c.cr1.modify(|_, w| w.start().set_bit());
                
                self.state = State::Wait(Event::Started);
            }

            fn start_receiving(&mut self) {
                // Send a START condition and set ACK bit
                self.i2c.cr1.modify(|_, w|
                    w.start().set_bit().ack().set_bit()
                );
                
                self.state = State::Wait(Event::Started);
            }

            fn stop(&mut self) {
                // Send STOP condition
                self.i2c.cr1.modify(|_, w| w.stop().set_bit());
                
                self.state = State::Idle;
            }

            fn polling_common_handler(&mut self) -> Result<(), Error> {
                use Event::*;

                if self.event_driven {
                    return Err(Error::WouldBlock);
                }

                // Wait until START condition was generated
                self.sr1_wait(|sr1| sr1.sb().bit_is_clear())?;

                // Set state to wait for MasterReady
                self.event_handler(Started)?;

                // Also wait until signalled we're master and everything is waiting for us
                self.wait(|| {
                    let sr2 = self.i2c.sr2.read();
                    Ok(sr2.msl().bit_is_clear() && sr2.busy().bit_is_clear())
                })?;

                // Set up current address, we're trying to talk to
                self.event_handler(MasterReady)?;

                // Wait until address was sent
                self.sr1_wait(|sr1| sr1.addr().bit_is_clear())?;

                // Clear condition by reading SR2
                self.event_handler(AddressSent)?;

                Ok(())
            }
        }

        impl<PINS> WriteRead for I2c<$I2CX, PINS> {
            type Error = Error;

            fn write_read(&mut self, addr: u8,
                tx_buf: &[u8], rx_buf: &mut [u8],
            ) -> Result<(), Self::Error> {
                self.event_driven = false;
                self.txn = Some(Transaction::write_read(
                    addr, tx_buf, rx_buf,
                ));
                self.txn_handler()
            }
        }

        impl<PINS> Write for I2c<$I2CX, PINS> {
            type Error = Error;

            fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
                self.event_driven = false;
                self.txn = Some(Transaction::write(addr, bytes));
                self.txn_handler()
            }
        }

        impl<PINS> Read for I2c<$I2CX, PINS> {
            type Error = Error;

            fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
                self.event_driven = false;
                self.txn = Some(Transaction::read(addr, buffer));
                self.txn_handler()
            }
        }

        impl<PINS> EventDriven for I2c<$I2CX, PINS> {
            fn enable_irqs(&mut self) {
                self.buffer_irq_enable();
                self.event_irq_enable();
                self.error_irq_enable();
            }
            fn disable_irqs(&mut self) {
                self.buffer_irq_disable();
                self.event_irq_disable();
                self.error_irq_disable();
            }        

            fn start_transaction(&mut self, txn: Transaction) -> Result<(),Error> {
                self.event_driven = true;
                self.txn = Some(txn);
                self.txn_handler()                
            }
            fn finish_transaction(&mut self) -> Result<Transaction,Error> {
                self.txn.take().ok_or(Error::General)
            }

            fn is_transaction(&mut self) -> bool {
                self.txn.is_some()
            }

            fn check_event(&mut self) -> Result</*is_finish:*/bool,Error> {
                use {State::*, Event::*};
                match self.state {
                    Wait(evt) => {
                        match evt {
                            Started => {
                                // Wait until START condition was generated
                                let sr1 = self.sr1_read()?;
                                if sr1.sb().bit_is_clear() {
                                    return Err(Error::WouldBlock);
                                }
                            },
                            MasterReady => {
                                // Wait until signalled we're master and everything is waiting for us
                                let sr2 = self.i2c.sr2.read();
                                if sr2.msl().bit_is_clear() && sr2.busy().bit_is_clear() {
                                    return Err(Error::WouldBlock);
                                }
                            },
                            AddressSent => {
                                // Wait until address was sent
                                let sr1 = self.sr1_read()?;
                                if sr1.addr().bit_is_clear() {
                                    return Err(Error::WouldBlock);
                                }
                            },
                            ReadyForSend => {
                                // Wait until we're ready for sending
                                let sr1 = self.sr1_read()?;
                                if sr1.tx_e().bit_is_clear() {
                                    return Err(Error::WouldBlock);
                                }
                            },
                            Sent => {
                                // While until byte is transferred
                                let sr1 = self.sr1_read()?;
                                if sr1.btf().bit_is_clear() {
                                    return Err(Error::WouldBlock);
                                }
                            },
                            ReadyForRecv => {
                                // Wait until we're ready for receiving
                                let sr1 = self.sr1_read()?;
                                if sr1.rx_ne().bit_is_clear() {
                                    return Err(Error::WouldBlock);
                                }
                            },
                        }
                        return self.event_handler(evt);
                    },
                    Idle => {  },
                }
                Err(Error::WouldBlock)
            }

            fn event_handler(&mut self, evt: Event) -> Result</*is_finish:*/bool, Error> {
                use {State::*, Event::*};
                match self.state {
                    Wait(wait_evt) => {
                        if evt != wait_evt {
                            panic!();
                            // return Err(General);
                        }
                        match evt {
                            Started => {
                                self.state = Wait(MasterReady);
                            },
                            MasterReady => {
                                let mode = self.txn_mode()? as u32;
                                // Set up current address, we're trying to talk to
                                self.i2c.dr.write(|w| unsafe {
                                    w.bits((u32::from(self.txn_addr()) << 1) + mode)
                                });
                                self.state = Wait(AddressSent);
                            },
                            AddressSent => {
                                // Clear condition by reading SR2
                                self.i2c.sr2.read();

                                match self.txn_mode()? {
                                    0 /*tx*/ => {
                                        self.state = Wait(ReadyForSend);
                                    },
                                    1 /*rx*/ => {
                                        self.state = Wait(ReadyForRecv);
                                    },
                                    _ => {}
                                }
                            },
                            ReadyForSend => {
                                if let Some(byte) = self.txn_next_byte_for_send()? {
                                    self.i2c.dr.write(|w| unsafe { w.bits(u32::from(byte)) });
                                    self.state = Wait(Sent);
                                }
                            },
                            Sent => {
                                if self.txn_need_sending() {
                                    self.state = Wait(ReadyForSend);
                                } else if self.event_driven && self.txn_need_receiving() {
                                    self.start_receiving();
                                } else {
                                    self.stop();
                                    return Ok(true);
                                }
                            },
                            ReadyForRecv => {
                                let byte = self.i2c.dr.read().bits() as u8;
                                if let Some(finish) = self.txn_save_recv_byte(byte)? {
                                    if finish {
                                        self.stop();
                                        return Ok(true);
                                    } else {
                                        self.state = Wait(ReadyForRecv);
                                    }
                                }
                            },
                        }
                    },
                    _ => {}
                }
                Ok(false)
            }
        }

        impl I2cExt<$I2CX> for $I2CX {
            fn i2c<PINS, T>(self, pins: PINS, speed: T, rcc: &mut Rcc) -> I2c<$I2CX, PINS>
            where
                PINS: Pins<$I2CX>,
                T: Into<Hertz>,
            {
                I2c::$i2cx(self, pins, speed.into(), rcc)
            }
        }
    };
}

pub trait I2cExt<I2C> {
    fn i2c<PINS, T>(self, pins: PINS, speed: T, rcc: &mut Rcc) -> I2c<I2C, PINS>
    where
        PINS: Pins<I2C>,
        T: Into<Hertz>;
}

#[derive(Copy,Clone,Debug,PartialEq)]
pub enum Event {
    Started, // START condition was generated
    MasterReady, // We're master and everything is waiting for us
    AddressSent, // Address was sent
    ReadyForSend, // We're ready for sending byte
    Sent, // Byte is transferred
    ReadyForRecv, // We're ready for receiving byte
}
enum State {
    Idle,
    Wait(Event),
}

pub struct Transaction {
    pub addr: u8,
    tx_buf: Option<(*const u8, usize)>,
    rx_buf: Option<(*mut u8, usize)>,
    tx_counter: usize,
    rx_counter: usize,
}
impl Transaction {
    fn new(addr: u8, tx_buf: Option<(*const u8, usize)>, rx_buf: Option<(*mut u8, usize)>) -> Self {
        Self {
            addr, tx_buf, rx_buf,
            tx_counter: 0, rx_counter: 0,
        }
    }
    pub fn write(addr: u8, buf: &[u8]) -> Self {
        Self::new(addr, Some((buf.as_ptr(),buf.len())), None)
    }
    pub fn read(addr: u8, buf: &mut [u8]) -> Self {
        Self::new(addr, None, Some((buf.as_mut_ptr(),buf.len())))
    }
    pub fn write_read(addr: u8, tx_buf: &[u8], rx_buf: &mut [u8]) -> Self {
        Self::new(addr, 
            Some((tx_buf.as_ptr(),tx_buf.len())), 
            Some((rx_buf.as_mut_ptr(),rx_buf.len()))
        )
    }

    pub fn write_on(&self) -> bool {
        self.tx_buf.is_some()
    }
    pub fn read_on(&self) -> bool {
        self.rx_buf.is_some()
    }

    pub fn is_finish(&self) -> bool {
        !(self.need_sending() || self.need_receiving())
    }

    fn mode(&self) -> Option<u8> {
        if self.need_sending() {
            Some(0)
        } else if self.need_receiving() {
            Some(1)
        } else {
            None
        }
    }

    fn need_sending(&self) -> bool {
        if let Some((_ptr,len)) = self.tx_buf.as_ref() {
            self.tx_counter < *len
        } else {
            false
        }
    }
    fn need_receiving(&self) -> bool {
        if let Some((_ptr,len)) = self.rx_buf.as_ref() {
            self.rx_counter < *len
        } else {
            false
        }
    }
    fn next_byte_for_send(&mut self) -> Option<u8> {
        if let Some((ptr,len)) = self.tx_buf.as_ref() {
            if self.tx_counter >= *len {
                None
            } else {
                let byte = unsafe { *ptr.offset(self.tx_counter as isize) };
                self.tx_counter += 1;
                Some(byte)
            }
        } else {
            None
        }
    }
    fn save_recv_byte(&mut self, byte: u8) -> Option<bool> {
        if let Some((ptr,len)) = self.rx_buf.as_mut() {
            if self.rx_counter >= *len {
                None
            } else {
                unsafe { *ptr.offset(self.rx_counter as isize) = byte; }
                self.rx_counter += 1;
                Some(self.rx_counter >= *len)
            }
        } else {
            None
        }
    }
}

pub trait EventDriven {
    fn enable_irqs(&mut self);
    fn disable_irqs(&mut self);

    fn start_transaction(&mut self, txn: Transaction) -> Result<(),Error>;
    fn finish_transaction(&mut self) -> Result<Transaction,Error>;
    fn is_transaction(&mut self) -> bool;

    fn check_events(&mut self) -> Result</*is_finish:*/bool,Error> {
        while !self.check_event()? {}
        Ok(true)
    }

    fn check_event(&mut self) -> Result</*is_finish:*/bool,Error>;
    fn event_handler(&mut self, evt: Event) -> Result</*is_finish:*/bool, Error>;
}

i2c!(I2C1, i2c1, i2c1en, i2c1rst);
i2c!(I2C2, i2c2, i2c2en, i2c2rst);
