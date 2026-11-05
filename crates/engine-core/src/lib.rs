//! Pure, deterministic engine physics. No IO, no threads.
//!
//! SI units everywhere (m, kg, s, Pa, K, rad); conversions live in [`units`].

pub mod air_cleaner;
pub mod combustion;
pub mod config;
pub mod drivetrain;
pub mod engine;
pub mod engine_bay;
pub mod exhaust;
pub mod exhaust_fv;
pub mod friction;
pub mod fuel;
pub mod fv;
pub mod gas;
pub mod geometry;
pub mod intake_fv;
pub mod jet_noise;
pub mod layout;
pub mod lining;
pub mod listener;
pub mod mechanical;
pub mod narrow_tube;
pub mod pipe_heat;
pub mod silencer2d;
pub mod sound;
pub mod structure;
pub mod thermo;
pub mod turbo;
pub mod units;
pub mod valve;
pub mod valve_boundary;
pub mod waveguide;
