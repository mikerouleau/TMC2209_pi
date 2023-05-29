mod tmc2209;

fn main() {
    let mut tmc = tmc2209::TMC2209::new("/dev/serial0", 0x0, 115_200u32);
    let reg = 0x6;
    let received = tmc.read_reg(reg);
    for i in received {
        println!("Received byte: {}", i);
    }
}
