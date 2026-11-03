//! Laying a pipe of a given length in 3D. The sim's pipes are 1D (a length and a bore): their path
//! is a drawing choice, kept compact — out along one direction, then back and forth on lanes side
//! by side, like a folded hose — and exactly as long as the pipe.

use glam::Vec3;

/// One straight piece of a laid pipe.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Run {
    pub from: Vec3,
    pub to: Vec3,
}

#[cfg(test)]
impl Run {
    pub fn length(&self) -> f32 {
        (self.to - self.from).length()
    }
}

/// Lay `length` metres from `start`: runs of at most `max_run` along `first_dir`, alternating its
/// sense, each next lane `lane` further along `fold_dir` (the legs between lanes count towards the
/// length). Appends the runs to `out` and returns the end. Nothing for a length ≤ 0.
pub fn route(
    start: Vec3,
    first_dir: Vec3,
    fold_dir: Vec3,
    lane: f32,
    length: f32,
    max_run: f32,
    out: &mut Vec<Run>,
) -> Vec3 {
    let along = first_dir.normalize_or_zero();
    let across = fold_dir.normalize_or_zero();
    let max_run = max_run.max(1e-3);
    let lane = lane.max(1e-4);
    let mut left = if length.is_finite() {
        length.max(0.0)
    } else {
        0.0
    };
    let mut at = start;
    let mut sense = 1.0;
    while left > 1e-9 {
        let run = left.min(max_run);
        let to = at + sense * run * along;
        out.push(Run { from: at, to });
        at = to;
        left -= run;
        if left <= 1e-9 {
            break;
        }
        let leg = left.min(lane);
        let to = at + leg * across;
        out.push(Run { from: at, to });
        at = to;
        left -= leg;
        sense = -sense;
    }
    at
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lay(length: f32) -> (Vec<Run>, Vec3) {
        let mut out = Vec::new();
        let end = route(
            Vec3::ZERO,
            Vec3::NEG_Y,
            Vec3::X,
            0.06,
            length,
            0.8,
            &mut out,
        );
        (out, end)
    }

    #[test]
    fn the_runs_add_up_to_the_length() {
        for l in [0.3, 1.7, 4.25] {
            let (runs, _) = lay(l);
            let total: f32 = runs.iter().map(Run::length).sum();
            assert!((total - l).abs() < 1e-4, "{l}: {total}");
        }
    }

    #[test]
    fn a_short_pipe_is_one_straight_run() {
        let (runs, end) = lay(0.05);
        assert_eq!(runs.len(), 1);
        assert!((end - Vec3::new(0.0, -0.05, 0.0)).length() < 1e-7, "{end}");
    }

    #[test]
    fn a_long_pipe_folds_inside_its_box() {
        // 6 m in runs of 0.8 and legs of 0.06: 7 lanes (6.42 m with their legs ≥ 6),
        // so x ≤ 7·0.06 and y within one run of the start
        let (runs, _) = lay(6.0);
        for r in &runs {
            for p in [r.from, r.to] {
                assert!(p.y <= 1e-6 && p.y >= -0.8 - 1e-6, "{p}");
                assert!(p.x >= -1e-6 && p.x <= 7.0 * 0.06 + 1e-6, "{p}");
                assert_eq!(p.z, 0.0);
            }
        }
    }

    #[test]
    fn runs_are_continuous() {
        let (runs, end) = lay(4.25);
        for w in runs.windows(2) {
            assert_eq!(w[0].to, w[1].from);
        }
        assert_eq!(runs.last().map(|r| r.to), Some(end));
        assert_eq!(runs[0].from, Vec3::ZERO);
    }

    #[test]
    fn nothing_to_lay_gives_no_runs() {
        for l in [0.0, -1.0, f32::NAN] {
            let (runs, end) = lay(l);
            assert!(runs.is_empty());
            assert_eq!(end, Vec3::ZERO);
        }
    }
}
