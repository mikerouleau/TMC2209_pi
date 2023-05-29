use rppal::uart::{Parity, Uart};
use std::path::PathBuf;
use std::time::Instant;

#[allow(dead_code)]
pub struct TMC2209 {
    pub path: PathBuf,
    pub addr: u8,
    pub baud_rate: u32,
    pub uart: Uart,
    sync_byte: u8,
}

impl TMC2209 {
    pub fn new(path: &str, addr: u8, baud_rate: u32) -> Self {
        TMC2209 {
            path: path.into(),
            addr,
            baud_rate,
            uart: Uart::with_path(path, baud_rate, Parity::None, 8, 1).unwrap(),
            sync_byte: 0x05,
        }
    }

    pub fn read(&mut self, reg: u8) -> Vec<u8> {
        let crc = crc8_atm(&[self.sync_byte, self.addr, reg], 0);
        let frame = [self.sync_byte, self.addr, reg, crc];

        let mut buffer = [0u8; 1];
        let mut received = Vec::new();

        println!("send");
        self.uart.write(&frame).expect("not written");

        println!("recieve");
        let start = Instant::now();
        loop {
            // Fill the buffer variable with any incoming data.
            if self.uart.read(&mut buffer).unwrap() > 0 {
                received.push(buffer[0]);
                println!("Buffer byte: {}", buffer[0]);
            }

            if (received.len() == 12) || (start.elapsed().as_micros() > 8 * 20000) {
                break;
            }
        }
        return received;
    }
}

fn crc8_atm(datagram: &[u8], initial_value: u8) -> u8 {
    let mut crc: u8 = initial_value;

    for bytey in datagram {
        let mut byte = *bytey;
        for _ in 0..8 {
            if (crc >> 7) ^ (byte & 0x01) != 0 {
                crc = ((crc << 1) ^ 0x07) & 0xFF;
            } else {
                crc = (crc << 1) & 0xFF;
            }
            byte = byte >> 1;
        }
    }
    crc
}
pub struct TMC2209Addr;

// https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf
#[allow(dead_code)]
impl TMC2209Addr {
    const WRITE_OFFSET: u32 = 0x80;

    // Registers
    pub const GCONF: u32 = 0x00;
    const GSTAT: u32 = 0x01;
    const IFCNT: u32 = 0x02;
    const IOIN: u32 = 0x06;
    const IHOLD_IRUN: u32 = 0x10;
    const TSTEP: u32 = 0x12;
    const VACTUAL: u32 = 0x22;
    const TCOOLTHRS: u32 = 0x14;
    const SGTHRS: u32 = 0x40;
    const SG_RESULT: u32 = 0x41;
    const COOLCONF: u32 = 0x42;
    const MSCNT: u32 = 0x6A;
    const MSCURACT: u32 = 0x6B;
    const CHOPCONF: u32 = 0x6C;
    const DRVSTATUS: u32 = 0x6F;
    const PWMCONF: u32 = 0x70;
    const PWM_SCALE: u32 = 0x71;
    const PWM_AUTO: u32 = 0x72;

    //bitmasks
    //GCONF
    const I_SCALE_ANALOG: u32 = 1 << 0;
    const INTERNAL_RSENSE: u32 = 1 << 1;
    const EN_SPREADCYCLE: u32 = 1 << 2;
    const SHAFT: u32 = 1 << 3;
    const INDEX_OTPW: u32 = 1 << 4;
    const INDEX_STEP: u32 = 1 << 5;
    const PDN_DISABLE: u32 = 1 << 6;
    const MSTEP_REG_SELECT: u32 = 1 << 7;
    const MULTISTEP_FILT: u32 = 1 << 8;
    const TEST_MODE: u32 = 1 << 9;

    //GSTAT
    const RESET: u32 = 1 << 0;
    const DRV_ERR: u32 = 1 << 1;
    const UV_CP: u32 = 1 << 2;

    //IFCNT
    const INTERFACE_CNT: u32 = 1 << 0;

    //NODECONF
    // const UV_CP: u32 = 1 << 8; // mask should be 11..8

    // Ignore OTP_PROG, FACTORY_CONF for now
    // Implement later if neededs

    // IHOLD_RUN
    const IHOLD: u32 = 31 << 0;
    const IRUN: u32 = 31 << 8;
    const IHOLDDELAY: u32 = 15 << 16;

    // Ignore TPOWER_DOWN
    // None required for TSTEP
    // Ignore TPWMTHRS
    // None required for VACTUAL

    //COOLCONF
    const SEMIN: u32 = 15 << 0;
    const SEUP: u32 = 3 << 5;
    const SEMAX: u32 = 15 << 8;
    const SEDN: u32 = 3 << 13;
    const SEIMIN: u32 = 1 << 15;

    // None required for MSCNT, MSCURACT

    //CHOPCONF
    const TOFF: u32 = 15 << 0;
    const HSTRT: u32 = 7 << 4;
    const HEND: u32 = 15 << 7;
    const TBL: u32 = 3 << 15;
    const VSENSE: u32 = 1 << 17;
    const MRES: u32 = 15 << 24;
    const INTPOL: u32 = 1 << 28;
    const DEDEG: u32 = 1 << 29;
    const DISS2G: u32 = 1 << 30;
    const DISS2VS: u32 = 1 << 31;

    //DRV_STATUS
    const OTPW: u32 = 1 << 0;
    const OT: u32 = 1 << 1;
    const S2GA: u32 = 1 << 2;
    const S2GB: u32 = 1 << 3;
    const S2VA: u32 = 1 << 4;
    const S2VB: u32 = 1 << 5;
    const OLA: u32 = 1 << 6;
    const OLB: u32 = 1 << 7;
    const T120: u32 = 1 << 8;
    const T143: u32 = 1 << 9;
    const T150: u32 = 1 << 10;
    const T157: u32 = 1 << 11;
    const CS_ACTUAL: u32 = 31 << 16;
    const STEALTH: u32 = 1 << 30;
    const STST: u32 = 1 << 31;

    //PWMCONF
    const PWM_OFS: u32 = 255 << 0;
    const PWM_GRAD: u32 = 255 << 8;
    const PWM_FREQ: u32 = 3 << 16;
    const PWM_AUTOSCALE: u32 = 1 << 19;
    const FREEWHEEL: u32 = 3 << 20;
    const PWM_REG: u32 = 15 << 24;
    const PWM_LIM: u32 = 15 << 28;
}
