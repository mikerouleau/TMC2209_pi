use rppal::uart::{Parity, Uart};
use std::{thread, time};

fn main() {
    println!("Hello, world!");

    let baud_rate = 115_200u32;
    let mut uart = Uart::with_path("/dev/serial0", baud_rate, Parity::None, 8, 1).unwrap();
    let sync_byte = 0x5;
    let addr = 0x0;
    let reg = 0x6;
    let crc = crc8_atm(&[sync_byte, addr, reg], 0);
    let frame = [sync_byte, addr, reg, crc];

    let mut buffer = [0u8; 1];

    println!("send");
    uart.write(&frame).expect("not written");

    // thread::sleep(time::Duration::from_micros(165));
    println!("recieve");
    loop {
        // Fill the buffer variable with any incoming data.
        if uart.read(&mut buffer).unwrap() > 0 {
            println!("Received byte: {}", buffer[0]);
        }
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
