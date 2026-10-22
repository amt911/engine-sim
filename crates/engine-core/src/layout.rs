//! How the cylinders stand: one bank (inline), or two banks at an angle (V; 180° = flat or
//! boxer). Drawn by the app; the firing timing still comes from the firing order.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Layout {
    /// Angle between the two banks, degrees: 0 = inline (one bank), 60–120 = V, 180 = flat.
    pub bank_angle_deg: f64,
    /// Cylinders (1-based) of each of the two banks, front to back. Empty = the first half
    /// on one bank and the rest on the other.
    pub banks: Vec<Vec<usize>>,
}

impl Layout {
    /// (bank, position along the bank) of each cylinder, in cylinder order. Cylinders a bank
    /// list leaves out go after the listed ones on bank 0.
    pub fn place(&self, cylinders: usize) -> Vec<(usize, usize)> {
        if self.bank_angle_deg == 0.0 {
            return (0..cylinders).map(|i| (0, i)).collect();
        }
        if self.banks.is_empty() {
            let first = cylinders.div_ceil(2);
            return (0..cylinders)
                .map(|i| if i < first { (0, i) } else { (1, i - first) })
                .collect();
        }
        let mut place = vec![None; cylinders];
        for (b, bank) in self.banks.iter().take(2).enumerate() {
            for (k, &c) in bank.iter().enumerate() {
                if (1..=cylinders).contains(&c) {
                    place[c - 1] = Some((b, k));
                }
            }
        }
        let mut next = self.banks.first().map_or(0, Vec::len);
        place
            .into_iter()
            .map(|p| {
                p.unwrap_or_else(|| {
                    next += 1;
                    (0, next - 1)
                })
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inline_is_one_bank_in_cylinder_order() {
        let l = Layout::default();
        assert_eq!(l.bank_angle_deg, 0.0);
        assert_eq!(l.place(4), vec![(0, 0), (0, 1), (0, 2), (0, 3)]);
    }

    #[test]
    fn a_v_without_banks_splits_the_cylinders_in_halves() {
        let l = Layout {
            bank_angle_deg: 90.0,
            banks: vec![],
        };
        assert_eq!(l.place(4), vec![(0, 0), (0, 1), (1, 0), (1, 1)]);
        // odd counts: the extra one on the first bank
        assert_eq!(l.place(3), vec![(0, 0), (0, 1), (1, 0)]);
    }

    #[test]
    fn named_banks_place_each_cylinder_in_its_bank_in_listed_order() {
        // GM numbering: odd cylinders on the left bank, even on the right
        let l = Layout {
            bank_angle_deg: 90.0,
            banks: vec![vec![1, 3, 5, 7], vec![2, 4, 6, 8]],
        };
        let p = l.place(8);
        assert_eq!(p[0], (0, 0));
        assert_eq!(p[1], (1, 0));
        assert_eq!(p[6], (0, 3));
        assert_eq!(p[7], (1, 3));
    }

    #[test]
    fn layout_round_trips_and_old_files_are_inline() {
        let d: Layout = serde_yaml_ng::from_str("{}").unwrap();
        assert_eq!(d, Layout::default());
        let l = Layout {
            bank_angle_deg: 72.0,
            banks: vec![vec![1, 3], vec![2, 4]],
        };
        let j: Layout = serde_json::from_str(&serde_json::to_string(&l).unwrap()).unwrap();
        let y: Layout = serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&l).unwrap()).unwrap();
        assert_eq!((j, y), (l.clone(), l));
    }

    #[test]
    fn cylinders_no_bank_names_go_after_the_first_bank() {
        // a V with 1–2 | 3 named: 4 and 5 follow on bank 0, after its two
        let l = Layout {
            bank_angle_deg: 90.0,
            banks: vec![vec![1, 2], vec![3]],
        };
        assert_eq!(l.place(5), vec![(0, 0), (0, 1), (1, 0), (0, 2), (0, 3)]);
    }
}
