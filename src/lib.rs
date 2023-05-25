#![no_std]

pub use stm32h7xx_hal as hal;


mod common;
mod net;

pub mod version {
	include!(concat!(env!("OUT_DIR"), "/version_info.rs"));

	pub const VERSION: (i32, i32, i32, i32, i32, i32) = (
		VERSION_YEAR, VERSION_MONTH, VERSION_DAY, 
		VERSION_HOUR, VERSION_MINUTE, VERSION_SECOND);
}


pub mod prelude {
	pub use crate::hal::prelude::*;
	pub use crate::common::*;
	pub use crate::net::*;
}