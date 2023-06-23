#![cfg_attr(feature = "nightly", feature(error_in_core))]
#![cfg_attr(feature = "nightly", feature(panic_info_message))]
#![no_std]

pub mod gpio;
pub mod serial;
pub mod timer;
#[cfg(feature = "usb")]
pub mod usb;

// export peripherals
pub use serial::Serial;
pub use timer::Timer;
#[cfg(feature = "usb")]
pub use usb::{Usb0, Usb1, Usb2};

// re-export dependencies
#[cfg(feature = "usb")]
pub use smolusb;

pub use lunasoc_pac as pac;

pub use embedded_hal as hal;
pub use embedded_hal_0 as hal_0;
pub(crate) use embedded_hal_nb as hal_nb;

pub use nb;
