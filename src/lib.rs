#![no_std]

pub use stm32h7xx_hal as hal;


mod common;
mod net;

pub mod prelude {
	pub use crate::hal::prelude::*;
	pub use crate::common::*;
	pub use crate::net::*;
}