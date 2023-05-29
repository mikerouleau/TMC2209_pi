# TMC2209_pi - A TMC2209 Stepper Motor Driver Interface for Raspberry Pi

[![MIT licensed](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

TMC2209_pi requires Raspberry Pi OS or a similar Linux distribution on some Raspberry Pi hardware.

This library is still under development and the API may still change. Pull requests are welcome.

## Usage

Add a dependency for `tmc2209_pi` to your `Cargo.toml` using `cargo add tmc2209_pi`, or by adding the following line to your dependencies section.

```toml
[dependencies]
tmc2209_pi = "0.14.1"
```

## Examples

This example demonstrates the movement of a stepper motor to a specific location using a combination of the UART and GPIO interfaces on the Raspberry Pi.

```rust
use tmc2209_pi::TMC2209;

fn main() {
    let mut tmc = TMC2209::new("/dev/serial0", 0x0, 115_200, 16, 20, 21, 26).unwrap();

    tmc.go_to_position(400).unwrap();
}
```