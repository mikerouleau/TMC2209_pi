use rppal::uart::{Parity, Uart};
use std::{thread, time};

fn main() {
    println!("Hello, world!");

    let baud_rate = 115_200u32;
    let mut uart = Uart::with_path("/dev/serial0", baud_rate, Parity::None, 8, 1).unwrap();
    let reg = 0x6;
    let crc = 0xe8;
    let frame = [0x55, 0x0, reg, crc];

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
