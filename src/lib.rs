//! This crate provides basic control for [`TMC2209`] stepper motor drivers using Rasberry Pi UART and GPIO interfaces.
//!
//! Note that this crate is still experimental and is not guaranteed to be stable or functional for all use cases.
//!
//! [`TMC2209`]: https://www.trinamic.com/products/integrated-circuits/details/tmc2209-la/

#![allow(
    clippy::doc_markdown,
    clippy::module_name_repetitions,
    clippy::must_use_candidate,
    clippy::option_if_let_else,
    clippy::redundant_else,
    clippy::manual_map,
    clippy::missing_safety_doc,
    clippy::missing_errors_doc
)]
#![warn(missing_docs)]
#![warn(rust_2018_idioms)]

/// interface for control of TMC2209 stepper motor driver on Rasberry Pi.
pub mod tmc2209;
