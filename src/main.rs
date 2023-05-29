mod tmc2209;
// use packed_struct::prelude::*;
// use std::thread;
// use std::time::Duration;

fn main() {
    let mut tmc = tmc2209::TMC2209::new("/dev/serial0", 0x0, 115_200u32, 16, 20, 21, 26).unwrap();

    tmc.reset_position();

    println!("position: {:}", tmc.position);

    let chopchop = tmc.get_chopconf().unwrap();
    println!("mres: {:}", chopchop.mres);

    tmc.set_rpm(10).unwrap();
    tmc.go_to_position(400).unwrap();
    tmc.set_rpm(50).unwrap();
    tmc.go_to_position(0).unwrap();
}
