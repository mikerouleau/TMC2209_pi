// use rppal::uart::{Parity, Uart};
use std::time::Instant;
// use std::{thread, time};

mod tmc2209;

fn main() {
    println!("Hello, world!");

    let mut tmc = tmc2209::TMC2209::new("/dev/serial0", 0x0, 115_200u32);
    // let baud_rate = 115_200u32;
    // let mut uart = Uart::with_path("/dev/serial0", baud_rate, Parity::None, 8, 1).unwrap();
    let sync_byte = 0x5;
    let addr = 0x0;
    let reg = 0x6;
    let crc = crc8_atm(&[sync_byte, addr, reg], 0);
    let frame = [sync_byte, addr, reg, crc];

    let mut buffer = [0u8; 1];
    let mut received = Vec::new();

    println!("send");
    tmc.uart.write(&frame).expect("not written");

    println!("recieve");
    let start = Instant::now();
    loop {
        // Fill the buffer variable with any incoming data.
        if tmc.uart.read(&mut buffer).unwrap() > 0 {
            received.push(buffer[0]);
            println!("Buffer byte: {}", buffer[0]);
        }

        if (received.len() == 12) || (start.elapsed().as_micros() > 8 * 20000) {
            break;
        }
    }
    for i in received {
        println!("Received byte: {}", i);
    }
}

// struct Port {
//     serialport: String,
//     baudrate: u32,
// }
// struct TMC2209 {
//     addr: u32,
//     port: Port,
// }

fn crc8_atm(datagram: &[u8], initial_value: u8) -> u8 {
    let mut crc: u8 = initial_value;

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
