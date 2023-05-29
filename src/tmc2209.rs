use anyhow::{anyhow, Result};
use packed_struct::prelude::*;
use rppal::uart::{Parity, Uart};
use std::convert::TryInto;
use std::path::PathBuf;
use std::time::Instant;

mod err;
pub mod reg;
use crate::tmc2209::err::TMC2209Error;

pub struct TMC2209 {
    pub path: PathBuf,
    pub addr: u8,
    pub baud_rate: u32,
    uart: Uart,
    sync_byte: u8,
}

#[allow(dead_code)]
impl TMC2209 {
    pub fn new(path: &str, addr: u8, baud_rate: u32) -> Self {
        let mut uart = Uart::with_path(path, baud_rate, Parity::None, 8, 1).unwrap();
        uart.set_write_mode(true).expect("failed to set blocking");
        TMC2209 {
            path: path.into(),
            addr,
            baud_rate,
            uart,
            sync_byte: 0x05,
        }
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

    pub fn write_reg(&mut self, reg: u8, data: [u8; 4]) -> anyhow::Result<()> {
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
        self.uart.write(&frame)?;
        self.uart.drain()?;
        Ok(())
    }

    pub fn test_uart(&mut self) -> Result<()> {
        let response = self.read_reg(reg::IOIN::ADDR)?;
        let unpacked = reg::IOIN::unpack_from_slice(&response);

        if unpacked.is_ok() {
            let data = unpacked.unwrap();
            (data.zero1 == 0.into())
                .then_some(())
                .ok_or(TMC2209Error::UnexpectedResponse)?;
            (data.zero2 == 0.into())
                .then_some(())
                .ok_or(TMC2209Error::UnexpectedResponse)?;
            (data.pdn_uart == true)
                .then_some(())
                .ok_or(TMC2209Error::UnexpectedResponse)?;

            return Ok(());
        } else {
            return Err(anyhow!(TMC2209Error::Unknown));
        }
    }

    pub fn set_direction(&mut self, dir: bool) -> Result<(), anyhow::Error> {
        let mut packed = self.read_reg(reg::GCONF::ADDR)?;
        let mut unpacked = reg::GCONF::unpack_from_slice(&packed)?;
        unpacked.shaft = dir;
        packed = unpacked.pack()?;

        let prior_cnt = reg::IFCNT::unpack_from_slice(&self.read_reg(reg::IFCNT::ADDR)?)?.cnt;

        self.write_reg(reg::GCONF::ADDR + reg::WRITE_OFFSET, packed)?;

        let response = self.read_reg(reg::IFCNT::ADDR)?;
        let after_cnt = reg::IFCNT::unpack_from_slice(&response)?.cnt;

        if prior_cnt < after_cnt {
            println!("SUCCESS");
            return Ok(());
        } else {
            println!("FAIL");
            return Err(anyhow!(err::TMC2209Error::Unknown));
        }
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
