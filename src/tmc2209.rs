use rppal::uart::{Parity, Uart};
use std::convert::TryInto;
use std::path::PathBuf;
use std::time::Instant;

mod err;
mod reg;

#[allow(dead_code)]
pub struct TMC2209 {
    pub path: PathBuf,
    pub addr: u8,
    pub baud_rate: u32,
    uart: Uart,
    sync_byte: u8,
}

impl TMC2209 {
    pub fn new(path: &str, addr: u8, baud_rate: u32) -> Self {
        TMC2209 {
            path: path.into(),
            addr,
            baud_rate,
            uart: Uart::with_path(path, baud_rate, Parity::None, 8, 1).unwrap(),
            sync_byte: 0x05,
        }
    }

    pub fn read_reg(&mut self, reg: u8) -> Result<[u8; 4], err::TMC2209Error> {
        let crc = crc8_atm(&[self.sync_byte, self.addr, reg]);
        let frame = [self.sync_byte, self.addr, reg, crc];

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
                    return Err(err::TMC2209Error::CRCMismatch);
                }

                //arbitrarily long timeout, seems plently long in testing
            } else if start.elapsed().as_millis() > 15 {
                return Err(err::TMC2209Error::TimedOut);
            }
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
