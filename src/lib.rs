//! This crate provides basic control for [`TMC2209`] stepper motor drivers using Rasberry Pi UART and GPIO interfaces.
//!
//! Note that this crate is still experimental and is not guaranteed to be stable or functional for all use cases.
//!
//! [`TMC2209`]: https://www.trinamic.com/products/integrated-circuits/details/tmc2209-la/

#![allow(
    clippy::doc_markdown,
    clippy::module_name_repetitions,
    clippy::must_use_candidate,
    clippy::option_if_let_else,
    clippy::redundant_else,
    clippy::manual_map,
    clippy::missing_safety_doc,
    clippy::missing_errors_doc
)]
#![warn(missing_docs)]
#![warn(rust_2018_idioms)]

/// interface for control of TMC2209 stepper motor driver on Rasberry Pi.
use anyhow::{anyhow, Result};
use packed_struct::prelude::*;
use rppal::gpio::{Gpio, InputPin, OutputPin};
use rppal::uart::{Parity, Uart};
use std::path::PathBuf;
use std::thread;
use std::time::{Duration, Instant};

mod err;
mod reg;
use crate::err::TMC2209Error;

/// Motor rotation directions
#[derive(PartialEq, Copy, Clone)]
pub enum Direction {
    ///Clockwise
    CW,
    ///Counterclockwise
    CCW,
}

/// An instance of a TMC2209 driver to control
///
/// For complete control, UART and GPIO connections are required. Each driver may have different pins/addresses
///  Pin numbers are BCM numbers, which does often not correspond to physical pin numbers!
pub struct TMC2209 {
    /// PathBuf to the UART interface (e.g. at "/dev/serial0").
    pub path: PathBuf,
    /// UART address of the TMC2209 driver (typically 0x0 by default).
    pub addr: u8,
    /// UART baud rate. This value must be between 9600 and 500k (up to 5M with external clock). 115200 Baud is recommended as a default.
    pub baud_rate: u32,
    /// BCM GPIO pin controlling the step interface.
    pub step_pin: OutputPin,
    /// BCM GPIO pin controlling the dir interface.
    pub dir_pin: OutputPin,
    /// BCM GPIO pin controlling the en interface.
    pub en_pin: OutputPin,
    /// BCM GPIO pin reading the diag interface.
    pub diag_pin: InputPin,
    /// A counter to determine position of a stepper motor.
    pub position: i32,
    /// Motor direction of rotation.
    direction: Direction,
    /// UART instance commmunicating with the TMC2209.
    uart: Uart,
    /// A (near) arbitrary sync byte to start UART communication with the TMC2209.
    sync_byte: u8,
    /// Time in milliseconds between steps when using the step/dir interface.
    step_time: u64,
}

#[allow(dead_code)]
impl TMC2209 {
    /// Constructs a new TMC2209
    ///
    /// Use notes: Specify BCM pin numbers for GPIO connections `step_pin`,`dir_pin`, `en_pin`, and `diag_pin`. This interface should be allowed to mutate.
    ///
    /// Construction will test UART communication and tweak a couple registers to allow for more complete UART control.
    ///
    /// # Arguments
    ///
    /// * `path` - Path to the UART interface (e.g. "/dev/serial0").
    /// * `addr` - The TMC2209's UART address (typically 0x0 by default).
    /// * `baud_rate` - UART baud rate. 115200 is recommended as default.
    /// * `step_pin` - BCM number of the GPIO pin controlling the step interface.
    /// * `dir_pin` - BCM number of the GPIO pin controlling the dir interface.
    /// * `en_pin` - BCM number of the GPIO pin controlling the en interface.
    /// * `diag_pin` - BCM number of the  GPIO pin reading the diag interface.
    ///
    /// # Example
    ///
    /// ```
    /// use tmc2209_pi::TMC2209
    /// let mut tmc = TMC2209::new("/dev/serial0", 0x0, 115_200, 16, 20, 21, 26)
    /// ```
    pub fn new(
        path: &str,
        addr: u8,
        baud_rate: u32,
        step_pin: u8,
        dir_pin: u8,
        en_pin: u8,
        diag_pin: u8,
    ) -> Result<Self, anyhow::Error> {
        let mut uart = Uart::with_path(path, baud_rate, Parity::None, 8, 1)?;
        uart.set_write_mode(true).expect("failed to set blocking");
        uart.set_read_mode(8, Duration::from_millis(100))
            .expect("failed to set blocking");

        let mut tmc = TMC2209 {
            path: path.into(),
            addr,
            baud_rate,
            step_pin: Gpio::new()?.get(step_pin)?.into_output(),
            dir_pin: Gpio::new()?.get(dir_pin)?.into_output(),
            en_pin: Gpio::new()?.get(en_pin)?.into_output(),
            diag_pin: Gpio::new()?.get(diag_pin)?.into_input(),
            position: 0,
            direction: Direction::CCW,
            uart,
            sync_byte: 0x05,
            step_time: 5000, // todo make me actual step time, not step time/2
        };
        tmc.init()?;
        Ok(tmc)
    }

    fn init(&mut self) -> Result<(), anyhow::Error> {
        let gconf = self.get_gconf()?;
        // CW: dir pin = 0, shaft reg = 0
        if gconf.shaft == self.dir_pin.is_set_high() {
            self.direction = Direction::CW;
        } else {
            self.direction = Direction::CCW;
        }

        self.test_uart()?;
        self.set_mstep_reg_select(true)?;
        //todo: test gpio
        return Ok(());
    }

    fn read_reg(&mut self, reg: u8) -> Result<[u8; 4]> {
        let crc = crc8_atm(&[self.sync_byte, self.addr, reg]);
        let frame = [self.sync_byte, self.addr, reg, crc];

        self.uart.flush(rppal::uart::Queue::Both)?;
        self.uart.write(&frame).expect("Frame not written.");

        let mut buffer = [0u8; 1];
        let mut received = Vec::new();

        let start = Instant::now();
        loop {
            // Fill the buffer variable with any incoming data.
            if self.uart.read(&mut buffer).unwrap() > 0 {
                received.push(buffer[0]);
            }

            if received.len() == 12 {
                received.drain(0..4);
                let response: [u8; 8] = vec_to_array(received);
                if is_valid_crc(&response) {
                    let data: [u8; 4] = response[3..7].try_into().unwrap();
                    return Ok(data);
                } else {
                    return Err(anyhow!(err::TMC2209Error::CRCMismatch));
                }

                //arbitrarily long timeout, seems plently long in testing
            } else if start.elapsed().as_millis() > 15 {
                return Err(anyhow!(err::TMC2209Error::TimedOut));
            }
        }
    }

    fn write_reg(&mut self, reg: u8, data: [u8; 4]) -> anyhow::Result<()> {
        let crc = crc8_atm(&[
            self.sync_byte,
            self.addr,
            reg,
            data[0],
            data[1],
            data[2],
            data[3],
        ]);
        let frame = [
            self.sync_byte,
            self.addr,
            reg,
            data[0],
            data[1],
            data[2],
            data[3],
            crc,
        ];

        self.uart.flush(rppal::uart::Queue::Both)?;

        let prior_cnt = reg::IFCNT::unpack_from_slice(&self.read_reg(reg::IFCNT::ADDR)?)?.cnt;

        thread::sleep(Duration::from_micros(1)); // delay enhances reliability

        self.uart.write(&frame)?;
        self.uart.drain()?;

        thread::sleep(Duration::from_micros(1));

        let after_cnt = reg::IFCNT::unpack_from_slice(&self.read_reg(reg::IFCNT::ADDR)?)?.cnt;

        if prior_cnt < after_cnt {
            return Ok(());
        } else {
            return Err(anyhow!(err::TMC2209Error::WriteFailed));
        }
    }

    /// Test the UART interface by attempting to read some expected values from the TMC2209 register. If pdn_disable is false, it is set to true.
    ///
    /// # Example
    ///
    /// ```
    /// if tmc.test_uart().is_ok(){
    ///     println!("UART is ok!")
    /// } else {
    ///     println!("Something went wrong!")
    /// }
    /// ```
    pub fn test_uart(&mut self) -> Result<()> {
        let response = self.read_reg(reg::IOIN::ADDR)?;
        let unpacked = reg::IOIN::unpack_from_slice(&response)?;

        (unpacked.zero1 == 0.into())
            .then_some(())
            .ok_or(TMC2209Error::UnexpectedResponse)?;
        (unpacked.zero2 == 0.into())
            .then_some(())
            .ok_or(TMC2209Error::UnexpectedResponse)?;
        if unpacked.pdn_uart != true {
            self.set_pdn_disable(true)?;
        }

        self.uart.drain()?;
        //todo read from DRV_STATUS here too

        return Ok(());
    }

    fn get_gconf(&mut self) -> Result<reg::GCONF, anyhow::Error> {
        let packed = self.read_reg(reg::GCONF::ADDR)?;
        return Ok(reg::GCONF::unpack_from_slice(&packed)?);
    }

    fn set_shaft(&mut self, dir: bool) -> Result<(), anyhow::Error> {
        let mut packed = self.read_reg(reg::GCONF::ADDR)?;
        let mut unpacked = reg::GCONF::unpack_from_slice(&packed)?;
        unpacked.shaft = dir;
        packed = unpacked.pack()?;

        self.write_reg(reg::GCONF::ADDR + reg::WRITE_OFFSET, packed)?;
        Ok(())
    }

    fn set_pdn_disable(&mut self, disable: bool) -> Result<(), anyhow::Error> {
        let mut packed = self.read_reg(reg::GCONF::ADDR)?;
        let mut unpacked = reg::GCONF::unpack_from_slice(&packed)?;
        unpacked.pdn_disable = disable;
        packed = unpacked.pack()?;

        self.write_reg(reg::GCONF::ADDR + reg::WRITE_OFFSET, packed)?;
        Ok(())
    }

    fn set_mstep_reg_select(&mut self, disable_pins: bool) -> Result<(), anyhow::Error> {
        let mut packed = self.read_reg(reg::GCONF::ADDR)?;
        let mut unpacked = reg::GCONF::unpack_from_slice(&packed)?;
        unpacked.mstep_reg_select = disable_pins;
        packed = unpacked.pack()?;

        self.write_reg(reg::GCONF::ADDR + reg::WRITE_OFFSET, packed)?;

        Ok(())
    }

    fn set_vactual(&mut self, velocity: i32) -> Result<(), anyhow::Error> {
        let mut packed = self.read_reg(reg::VACTUAL::ADDR)?;
        let mut unpacked = reg::VACTUAL::unpack_from_slice(&packed)?;
        unpacked.vactual = velocity.into();
        packed = unpacked.pack()?;
        self.write_reg(reg::VACTUAL::ADDR + reg::WRITE_OFFSET, packed)?;
        Ok(())
    }

    //todo rename start() and stop(), read from self.rpm
    /// allows moving the motor by UART control.
    /// `velocity` range is given in +-(2^23)-1 [µsteps / t]
    /// This function does NOT increment the positon counter and should only be used for low-precision applications. The motor direction is also determined by the sign of `velocity`, not the value of self.direction.
    pub fn set_vel(&mut self, velocity: i32) -> Result<(), anyhow::Error> {
        self.set_vactual(velocity)
        //todo update position based on mscnt
    }

    fn set_en(&mut self, level: bool) -> Result<(), anyhow::Error> {
        if level {
            self.en_pin.set_high();
        } else {
            self.en_pin.set_low();
        }
        Ok(())
    }

    /// enables TMC2209 output by pulling the `en` GPIO pin high when `tf` is true. When  `tf` is false, en is lowered and output is disabled.
    pub fn enable_output(&mut self, tf: bool) -> Result<(), anyhow::Error> {
        return self.set_en(!tf);
    }

    /// Returns true if output is enabled. More specifically, it returns the state of the en GPIO pin, which enables TMC2209 output.
    pub fn is_output_enabled(&mut self) -> bool {
        self.en_pin.is_set_low()
    }

    /// Controls motor direction by changing the state of the dir GPIO pin.
    pub fn set_dir(&mut self, direction: Direction) -> Result<(), anyhow::Error> {
        if self.direction == direction {
        } else {
            if self.dir_pin.is_set_high() {
                self.dir_pin.set_low();
            } else {
                self.dir_pin.set_high();
            }
            self.direction = direction;
        }
        Ok(())
    }

    /// Returns the Direction of the motor shaft.
    pub fn get_dir(&mut self) -> Direction {
        return self.direction;
    }

    fn get_chopconf(&mut self) -> Result<reg::CHOPCONF, anyhow::Error> {
        let packed = self.read_reg(reg::CHOPCONF::ADDR)?;
        return Ok(reg::CHOPCONF::unpack_from_slice(&packed)?);
    }

    fn step(&mut self) {
        self.step_pin.set_high();
        thread::sleep(Duration::from_micros(self.step_time));
        self.step_pin.set_low();
        thread::sleep(Duration::from_micros(self.step_time));
        if self.dir_pin.is_set_low() {
            self.position += 1;
        } else {
            self.position -= 1;
        }
    }

    /// Resets the position counter to 0.
    pub fn reset_position(&mut self) {
        self.position = 0;
    }

    //todo monitor stallguard for failure
    /// Moves the motor to step `position`. Where exactly this is depends on microstep settings, stepper motor model, etc.
    pub fn go_to_position(&mut self, position: i32) -> Result<(), anyhow::Error> {
        let deactivate = if self.is_output_enabled() {
            false
        } else {
            self.enable_output(true)?;
            true
        };

        if position - self.position > 0 {
            self.set_dir(Direction::CW)?;
        } else {
            self.set_dir(Direction::CCW)?;
        }

        while self.position != position {
            self.step();
        }

        if deactivate {
            self.enable_output(false)?;
        }
        Ok(())
    }

    /// Directly set the time between steps (`step_time`) in milliseconds.
    pub fn set_step_time(&mut self, step_time: u32) -> Result<(), anyhow::Error> {
        let min_step_time = 500;
        if step_time < min_step_time {
            println!("max speed exceeded, using step_time = 1000");
            self.step_time = min_step_time.into();
        } else {
            self.step_time = (step_time / 2).into();
        }
        Ok(())
    }

    /// Sets the speed of the motor in terms of rev/m (`rpm`). This assumes 400 steps/rev, which may not be true for you. If you prefer, set the time between steps directly using `set_step_time(t)`.
    pub fn set_rpm(&mut self, rpm: u8) -> Result<(), anyhow::Error> {
        let nsteps_per_rev: u16 = 400;

        let step_time = 1000000 / (nsteps_per_rev as u32) * 60 / 2 / (rpm as u32);
        self.set_step_time(step_time)?;
        Ok(())
    }
}

fn crc8_atm(datagram: &[u8]) -> u8 {
    let mut crc = 0u8;

    for bytey in datagram {
        let mut byte = *bytey;
        for _ in 0..8 {
            if (crc >> 7) ^ (byte & 0x01) != 0 {
                crc = ((crc << 1) ^ 0x07) & 0xFF;
            } else {
                crc = (crc << 1) & 0xFF;
            }
            byte = byte >> 1;
        }
    }
    crc
}

// This is messy, but works. Needs optimization.
fn is_valid_crc(datagram: &[u8]) -> bool {
    let n = datagram.len();
    let expected_crc = datagram[n - 1];
    if expected_crc == crc8_atm(&datagram[0..n - 1]) {
        return true;
    } else {
        return false;
    }
}

fn vec_to_array<T, const N: usize>(v: Vec<T>) -> [T; N] {
    v.try_into()
        .unwrap_or_else(|v: Vec<T>| panic!("Expected a Vec of length {} but it was {}", N, v.len()))
}
