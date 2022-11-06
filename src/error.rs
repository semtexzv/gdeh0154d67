//! Currently this module is work in progress.
use thiserror_no_std::Error;

#[derive(Error, Debug)]
pub enum Error {
    #[error("unknown error")]
    Unknown,
}