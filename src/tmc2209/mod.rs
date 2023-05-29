use anyhow::{anyhow, Result};
use packed_struct::prelude::*;
use rppal::gpio::{Gpio, InputPin, OutputPin};
use rppal::uart::{Parity, Uart};
use std::convert::TryInto;
use std::path::PathBuf;
use std::thread;
use std::time::{Duration, Instant};

mod err;
pub mod reg;
use crate::tmc2209::err::TMC2209Error;

pub enum Direction {
    CW,
    CCW,
}
//Pins are BCM number!
pub struct TMC2209 {
    pub path: PathBuf,
    pub addr: u8,
    pub baud_rate: u32,
    pub step_pin: OutputPin,
    pub dir_pin: OutputPin, //remove me?
    pub en_pin: OutputPin,
    pub diag_pin: InputPin,
    pub position: i32,
    direction: Direction,
    uart: Uart,
    sync_byte: u8,
}

#[allow(dead_code)]
impl TMC2209 {
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

    pub fn read_reg(&mut self, reg: u8) -> Result<[u8; 4]> {
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

    pub fn enable_output(&mut self, tf: bool) -> Result<(), anyhow::Error> {
        return self.set_en(!tf);
    }

    pub fn is_output_enabled(&mut self) -> bool {
        self.en_pin.is_set_low()
    }

    pub fn set_dir(&mut self, level: bool) -> Result<(), anyhow::Error> {
        if level {
            self.dir_pin.set_high();
        } else {
            self.dir_pin.set_low();
        }
        Ok(())
    }

    pub fn get_dir(&mut self) -> bool {
        return self.dir_pin.is_set_high();
    }

    pub fn get_chopconf(&mut self) -> Result<reg::CHOPCONF, anyhow::Error> {
        let packed = self.read_reg(reg::CHOPCONF::ADDR)?;
        return Ok(reg::CHOPCONF::unpack_from_slice(&packed)?);
    }

    pub fn step(&mut self) {
        self.step_pin.set_high();
        thread::sleep(Duration::from_micros(700));
        self.step_pin.set_low();
        thread::sleep(Duration::from_micros(700));
        if self.dir_pin.is_set_low() {
            self.position += 1;
        } else {
            self.position -= 1;
        }
    }

    pub fn reset_position(&mut self) {
        self.position = 0;
    }

    //todo monitor stallguard for failure
    pub fn go_to_position(&mut self, position: i32) -> Result<(), anyhow::Error> {
        let deactivate = if self.is_output_enabled() {
            false
        } else {
            self.enable_output(true)?;
            true
        };

        if position - self.position > 0 {
            self.set_dir(false)?;
        } else {
            self.set_dir(true)?;
        }

        while self.position != position {
            self.step();
        }

        if deactivate {
            self.enable_output(false)?;
        }
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
