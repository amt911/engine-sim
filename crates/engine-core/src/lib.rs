//! Pure, deterministic engine physics. No IO, no threads.
//!
//! SI units everywhere (m, kg, s, Pa, K, rad); conversions live in [`units`].

pub mod combustion;
pub mod config;
pub mod drivetrain;
pub mod engine;
pub mod exhaust;
pub mod exhaust_fv;
pub mod fv;
pub mod gas;
pub mod geometry;
pub mod intake_fv;
pub mod layout;
pub mod lining;
pub mod listener;
pub mod sound;
pub mod units;
pub mod valve;
pub mod waveguide;
