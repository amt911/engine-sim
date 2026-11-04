//! How finely the 3D view is drawn, chosen from the measured frame time like the sim's quality
//! levels: a slow stretch steps it down at once, a long fast one steps it back up.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Detail {
    Low,
    Medium,
    High,
}

impl Detail {
    /// Segments round a cylinder mesh.
    /// Whether this level draws the realistic parts (Medium and High; Low keeps the primitives,
    /// the cheapest).
    pub fn realistic(self) -> bool {
        self != Detail::Low
    }

    pub fn segments(self) -> u32 {
        match self {
            Detail::Low => 12,
            Detail::Medium => 24,
            Detail::High => 48,
        }
    }
    fn down(self) -> Self {
        match self {
            Detail::High => Detail::Medium,
            _ => Detail::Low,
        }
    }
    fn up(self) -> Self {
        match self {
            Detail::Low => Detail::Medium,
            _ => Detail::High,
        }
    }
}

/// Frames over [`SLOW_MS`] in a row before stepping down; frames under [`FAST_MS`] in a row
/// before stepping up (ten times longer: dropping is urgent, climbing back can wait).
pub const SLOW_FRAMES: u32 = 30;
pub const FAST_FRAMES: u32 = 300;
/// A frame slower than this misses 60 fps with some margin; faster than this has room to spare.
pub const SLOW_MS: f32 = 18.0;
pub const FAST_MS: f32 = 10.0;

#[derive(Debug, Clone, PartialEq)]
pub struct DetailGovernor {
    pub detail: Detail,
    manual: Option<Detail>,
    slow: u32,
    fast: u32,
}

impl Default for DetailGovernor {
    fn default() -> Self {
        Self {
            detail: Detail::High,
            manual: None,
            slow: 0,
            fast: 0,
        }
    }
}

impl DetailGovernor {
    /// Feed one frame's time; the new detail when it changes.
    pub fn observe(&mut self, frame_ms: f32) -> Option<Detail> {
        if self.manual.is_some() || !frame_ms.is_finite() {
            return None;
        }
        if frame_ms > SLOW_MS {
            self.slow += 1;
            self.fast = 0;
        } else if frame_ms < FAST_MS {
            self.fast += 1;
            self.slow = 0;
        } else {
            self.slow = 0;
            self.fast = 0;
        }
        let next = if self.slow >= SLOW_FRAMES {
            self.detail.down()
        } else if self.fast >= FAST_FRAMES {
            self.detail.up()
        } else {
            return None;
        };
        self.slow = 0;
        self.fast = 0;
        (next != self.detail).then(|| {
            self.detail = next;
            next
        })
    }

    /// A detail chosen by hand (None = automatic again).
    pub fn set_manual(&mut self, d: Option<Detail>) {
        self.manual = d;
        if let Some(d) = d {
            self.detail = d;
        }
        self.slow = 0;
        self.fast = 0;
    }

    pub fn manual(&self) -> Option<Detail> {
        self.manual
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detail_steps_down_after_30_slow_frames() {
        let mut g = DetailGovernor::default();
        for _ in 0..29 {
            assert_eq!(g.observe(25.0), None);
        }
        assert_eq!(g.observe(25.0), Some(Detail::Medium));
        // a fresh count for the next step
        for _ in 0..29 {
            assert_eq!(g.observe(25.0), None);
        }
        assert_eq!(g.observe(25.0), Some(Detail::Low));
        for _ in 0..100 {
            assert_eq!(g.observe(25.0), None);
        }
    }

    #[test]
    fn one_ordinary_frame_restarts_the_count() {
        let mut g = DetailGovernor::default();
        for _ in 0..29 {
            g.observe(25.0);
        }
        assert_eq!(g.observe(14.0), None);
        for _ in 0..29 {
            assert_eq!(g.observe(25.0), None);
        }
        assert_eq!(g.observe(25.0), Some(Detail::Medium));
    }

    #[test]
    fn detail_waits_300_fast_frames_to_step_up() {
        let mut g = DetailGovernor::default();
        g.set_manual(Some(Detail::Low));
        g.set_manual(None);
        for _ in 0..299 {
            assert_eq!(g.observe(5.0), None);
        }
        assert_eq!(g.observe(5.0), Some(Detail::Medium));
    }

    #[test]
    fn manual_detail_wins() {
        let mut g = DetailGovernor::default();
        g.set_manual(Some(Detail::Medium));
        for _ in 0..1000 {
            assert_eq!(g.observe(50.0), None);
        }
        assert_eq!(g.detail, Detail::Medium);
        assert_eq!(g.manual(), Some(Detail::Medium));
    }

    #[test]
    fn segments_per_level() {
        assert_eq!(
            [Detail::Low, Detail::Medium, Detail::High].map(Detail::segments),
            [12, 24, 48]
        );
    }
}
