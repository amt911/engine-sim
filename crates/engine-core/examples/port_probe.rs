//! Probe: exhaust port flow and port pressure statistics, coast vs WOT.
use engine_core::engine::*;
fn main() {
    let dt = 1.0 / 192_000.0;
    for t in [0.0, 1.0] {
        let mut def = if std::env::args().nth(1).as_deref() == Some("lfa") {
            EngineDef::lfa_v10()
        } else {
            EngineDef::single()
        };
        if std::env::args().nth(3).as_deref() == Some("fv") {
            def.exhaust.model = engine_core::exhaust::ExhaustModel::Fv;
        }
        let rpm: f64 = std::env::args()
            .nth(2)
            .and_then(|a| a.parse().ok())
            .unwrap_or(6000.0);
        let mut e = Engine::new(def, rpm, 1);
        let c = Controls {
            throttle: t,
            hold_rpm: Some(rpm),
            ..Controls::default()
        };
        for _ in 0..(0.4 / dt) as usize {
            e.step(dt, &c);
        }
        let (mut pmin, mut pmax, mut amin, mut amax, mut cylmin, mut cylmax) =
            (f64::MAX, f64::MIN, f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        let mut evo = None;
        let mut last_lift = 1.0;
        for _ in 0..(0.02 / dt) as usize {
            let a = e.step(dt, &c);
            let v = e.cylinder(0);
            let pv = e.exhaust_port_gauge(0);
            pmin = pmin.min(pv);
            pmax = pmax.max(pv);
            amin = amin.min(a);
            amax = amax.max(a);
            cylmin = cylmin.min(v.pressure_pa);
            cylmax = cylmax.max(v.pressure_pa);
            if evo.is_none() && v.exhaust_lift_m > 0.0 && last_lift == 0.0 {
                evo = Some(v.pressure_pa);
            }
            last_lift = v.exhaust_lift_m;
        }
        println!(
            "throttle {t}: cyl {:.2}..{:.2} bar, at EVO {:.2} bar | port gauge {:.0}..{:.0} Pa | radiated {:.0}..{:.0}",
            cylmin / 1e5,
            cylmax / 1e5,
            evo.unwrap_or(0.0) / 1e5,
            pmin,
            pmax,
            amin,
            amax
        );
    }
}
