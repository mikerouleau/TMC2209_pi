use tmc2209_pi::TMC2209;

fn main() {
    let mut tmc = TMC2209::new("/dev/serial0", 0x0, 115_200, 16, 20, 21, 26).unwrap();

    tmc.reset_position();
    println!("zeroed position: {:}", tmc.position);

    tmc.go_to_position(400).unwrap();
    println!("position is now: {:}", tmc.position);

    tmc.set_rpm(20).unwrap();
    tmc.go_to_position(0).unwrap();
    println!("position is now: {:}", tmc.position);
}
