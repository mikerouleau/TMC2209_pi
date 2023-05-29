mod tmc2209;
use packed_struct::prelude::*;

fn main() {
    let mut tmc = tmc2209::TMC2209::new("/dev/serial0", 0x0, 115_200u32, 16, 20, 21, 26).unwrap();
    tmc.test_uart().expect("msg");
    // let reg = 0x06F;
    // let received = tmc.read_reg(reg).expect("Read Failed");
    // for i in received {
    //     println!("Received byte: {:08b}", i);
    // }
    // let unpacked =
    //     tmc2209::reg::IOIN::unpack_from_slice(&received).expect("failed to unpack response");

    // println!("VERSION: {:}", unpacked.version);

    // tmc.set_shaft(false).expect("errorrrorrr");
    tmc.reset_position();
    println!("position: {:}", tmc.position);

    tmc.enable_output(true).unwrap();
    tmc.go_to_position(-100).unwrap();
    tmc.go_to_position(200).unwrap();
    tmc.go_to_position(-200).unwrap();
    tmc.go_to_position(0).unwrap();
    println!("position: {:}", tmc.position);
}
