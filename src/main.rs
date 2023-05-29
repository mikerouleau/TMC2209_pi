mod tmc2209;
use packed_struct::prelude::*;

fn main() {
    let mut tmc = tmc2209::TMC2209::new("/dev/serial0", 0x0, 115_200u32);
    tmc.test_uart().expect("msg");
    let reg = 0x00;
    let received = tmc.read_reg(reg).expect("Read Failed");
    for i in received {
        println!("Received byte: {:08b}", i);
    }
    let unpacked =
        tmc2209::reg::IOIN::unpack_from_slice(&received).expect("failed to unpack response");

    println!("VERSION: {:}", unpacked.version);

    tmc.set_direction(false).expect("errorrrorrr")
}
