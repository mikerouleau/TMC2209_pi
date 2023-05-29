use rppal::uart::{Parity, Uart};
use std::path::PathBuf;
use std::time::Instant;

mod reg;

#[allow(dead_code)]
pub struct TMC2209 {
    pub path: PathBuf,
    pub addr: u8,
    pub baud_rate: u32,
    pub uart: Uart,
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

    pub fn read_reg(&mut self, reg: u8) -> Vec<u8> {
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
                if valid_crc_8byte(&received) {
                    received.drain(0..3);
                    received.pop();
                } else {
                    received = vec![];
                    println!("CRC Mismatch")
                }
                break;
                //arbitrarily long timeout, seems plently long in testing
            } else if start.elapsed().as_millis() > 15 {
                received = vec![];
                println!("Read Timeout!");
                break;
            }
        }

        return received;
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
fn valid_crc_8byte(datagram: &Vec<u8>) -> bool {
    let mut local_dg = datagram.clone();
    let expected_crc = local_dg.pop();
    let dg: [u8; 7] = vec_to_array(local_dg);
    if expected_crc == Some(crc8_atm(&dg)) {
        return true;
    } else {
        return false;
    }
}

use std::convert::TryInto;
use std::vec;

fn vec_to_array<T, const N: usize>(v: Vec<T>) -> [T; N] {
    v.try_into()
        .unwrap_or_else(|v: Vec<T>| panic!("Expected a Vec of length {} but it was {}", N, v.len()))
}
