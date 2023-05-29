use thiserror::Error;

#[allow(dead_code)]
#[derive(Error, Debug)]
pub enum TMC2209Error {
    #[error("CRC mismatch")]
    TimedOut,
    #[error("invalid CRC")]
    CRCMismatch,
    #[error("unknown TMC2209 error")]
    Unknown,
    #[error("unexpected response")]
    UnexpectedResponse,
}
