use packed_struct::prelude::*;

#[allow(dead_code)]
pub const WRITE_OFFSET: u8 = 0x80;

#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct GCONF {
    #[packed_field(bits = "0")]
    pub i_scale_analog: bool,
    #[packed_field(bits = "1")]
    pub internal_rsense: bool,
    #[packed_field(bits = "2")]
    pub en_spreadcycle: bool,
    #[packed_field(bits = "3")]
    pub shaft: bool,
    #[packed_field(bits = "4")]
    pub index_otpw: bool,
    #[packed_field(bits = "5")]
    pub index_step: bool,
    #[packed_field(bits = "6")]
    pub pdn_disable: bool,
    #[packed_field(bits = "7")]
    pub mstep_reg_select: bool,
    #[packed_field(bits = "8")]
    pub multistep_filt: bool,
    #[packed_field(bits = "9")]
    pub test_mode: bool,
}

#[allow(dead_code)]
impl GCONF {
    pub const ADDR: u8 = 0x00;
}

#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct GSTAT {
    #[packed_field(bits = "0")]
    pub reset: bool,
    #[packed_field(bits = "1")]
    pub drv_err: bool,
    #[packed_field(bits = "2")]
    pub uv_cp: bool,
}

#[allow(dead_code)]
impl GSTAT {
    pub const ADDR: u8 = 0x01;
}

#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct IFCNT {
    #[packed_field(bytes = "0")]
    pub cnt: u8,
}

#[allow(dead_code)]
impl IFCNT {
    pub const ADDR: u8 = 0x02;
}

#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct NODECONF {
    #[packed_field(bits = "8..=11")]
    pub send_delay: u8,
}

#[allow(dead_code)]
impl NODECONF {
    pub const ADDR: u8 = 0x03;
}

// ignore OTP programming for now

#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct IOIN {
    #[packed_field(bits = "0")]
    pub enn: bool,
    #[packed_field(bits = "1")]
    pub zero1: Integer<u8, packed_bits::Bits<1>>,
    #[packed_field(bits = "2")]
    pub ms1: bool,
    #[packed_field(bits = "3")]
    pub ms2: bool,
    #[packed_field(bits = "4")]
    pub diag: bool,
    #[packed_field(bits = "5")]
    pub zero2: Integer<u8, packed_bits::Bits<1>>,
    #[packed_field(bits = "6")]
    pub pdn_uart: bool,
    #[packed_field(bits = "7")]
    pub step: bool,
    #[packed_field(bits = "8")]
    pub spread_en: bool,
    #[packed_field(bits = "9")]
    pub dir: bool,
    #[packed_field(bytes = "3")]
    pub version: u8,
}

#[allow(dead_code)]
impl IOIN {
    pub const ADDR: u8 = 0x06;
}

// Ignore OTP_PROG, OTP_READ
// Implement later if needed

// Ignore FACTORY_CONF

#[allow(non_camel_case_types)]
#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct IHOLD_IRUN {
    #[packed_field(bits = "0..=4")]
    pub ihold: Integer<u8, packed_bits::Bits<5>>,
    #[packed_field(bits = "8..=12")]
    pub irun: Integer<u8, packed_bits::Bits<5>>,
    #[packed_field(bits = "16..=19")]
    pub ihold_delay: Integer<u8, packed_bits::Bits<4>>,
}

#[allow(dead_code)]
impl IHOLD_IRUN {
    pub const ADDR: u8 = 0x10;
}

#[allow(non_camel_case_types)]
#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct TPOWER_DOWN {
    #[packed_field(bytes = "0")]
    pub tpower_down: Integer<u8, packed_bits::Bits<8>>,
}

#[allow(dead_code)]
impl TPOWER_DOWN {
    pub const ADDR: u8 = 0x11;
}

#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct TSTEP {
    #[packed_field(bits = "0..=19")]
    pub tstep: Integer<u32, packed_bits::Bits<20>>,
}

#[allow(dead_code)]
impl TSTEP {
    pub const ADDR: u8 = 0x12;
}

#[allow(non_camel_case_types)]
#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct TPWM_THRS {
    #[packed_field(bits = "0..=19")]
    pub tpwm_thrs: Integer<u32, packed_bits::Bits<20>>,
}

#[allow(dead_code)]
impl TPWM_THRS {
    pub const ADDR: u8 = 0x13;
}

#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct VACTUAL {
    #[packed_field(bits = "0..=23")]
    pub tpwm_thrs: Integer<i32, packed_bits::Bits<24>>,
}

#[allow(dead_code)]
impl VACTUAL {
    pub const ADDR: u8 = 0x22;
}

#[allow(non_camel_case_types)]
#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct TCOOL_THRS {
    #[packed_field(bits = "0..=19")]
    pub tcool_thrs: Integer<u32, packed_bits::Bits<20>>,
}

#[allow(dead_code)]
impl TCOOL_THRS {
    pub const ADDR: u8 = 0x14;
}

#[allow(non_camel_case_types)]
#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct SG_THRS {
    #[packed_field(bits = "0..=7")]
    pub sg_thrs: Integer<u8, packed_bits::Bits<8>>,
}

#[allow(dead_code)]
impl SG_THRS {
    pub const ADDR: u8 = 0x40;
}

#[allow(non_camel_case_types)]
#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct SG_RESULT {
    #[packed_field(bits = "0..=9")] // bit 0 and 9 always 0
    pub sg_result: Integer<u16, packed_bits::Bits<10>>,
}

#[allow(dead_code)]
impl SG_RESULT {
    pub const ADDR: u8 = 0x41;
}

// #[allow(non_camel_case_types)]
// #[derive(PackedStruct)]
// #[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
// pub struct COOL_CONF {
//     #[packed_field(bits = "0..=9")]
//     pub see_table: Integer<u16, packed_bits::Bits<10>>,
// }

// #[allow(dead_code)]
// impl COOL_CONF {
//     pub const ADDR: u8 = 0x42;
// }

#[allow(non_camel_case_types)]
#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct MS_CNT {
    #[packed_field(bits = "0..=9")]
    pub ms_cnt: Integer<u16, packed_bits::Bits<10>>,
}

#[allow(dead_code)]
impl MS_CNT {
    pub const ADDR: u8 = 0x6A;
}

#[allow(non_camel_case_types)]
#[derive(PackedStruct)]
#[packed_struct(size_bytes = "4", bit_numbering = "lsb0", endian = "msb")]
pub struct MS_CUR_ACT {
    #[packed_field(bits = "0..=8")]
    pub cur_b: Integer<i16, packed_bits::Bits<9>>,
    #[packed_field(bits = "16..=24")]
    pub cur_a: Integer<i16, packed_bits::Bits<9>>,
}

#[allow(dead_code)]
impl MS_CUR_ACT {
    pub const ADDR: u8 = 0x6B;
}
