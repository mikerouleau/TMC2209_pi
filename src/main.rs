mod tmc2209;

fn main() {
    let mut tmc = tmc2209::TMC2209::new("/dev/serial0", 0x0, 115_200u32);
    let reg = 0x6F;
    let received = tmc.read_reg(reg).expect("Read Failed");
    for i in received {
        println!("Received byte: {:08b}", i);
    }
}
