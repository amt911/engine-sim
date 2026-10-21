//! Pure, deterministic engine physics. No IO, no threads.
//!
//! SI units everywhere (m, kg, s, Pa, K, rad); conversions live in [`units`].

pub mod combustion;
pub mod config;
pub mod engine;
pub mod gas;
pub mod geometry;
pub mod units;
pub mod valve;
