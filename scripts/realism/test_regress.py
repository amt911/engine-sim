"""regress.py compares every gated noise band (run: python -m unittest test_regress)."""
import os
import tempfile
import unittest

import rdata


def summary(sfx, wind, lowmid, hump):
    return (f"k20-{sfx} (i4, 6 speed bins): FAIL (x)   score / threshold: order 1.00/1.51, "
            f"wind {wind}/-21.1 dB, lowmid {lowmid}/-9.0 dB, hump {hump}/3.0 dB\n")


class RegressTest(unittest.TestCase):
    def run_pair(self, base, new):
        with tempfile.TemporaryDirectory() as d:
            os.makedirs(os.path.join(d, "renders"))
            for sfx, line in (("base", base), ("new", new)):
                with open(os.path.join(d, "renders", f"gates-{sfx}.txt"), "w") as f:
                    f.write(line)
            old, rdata.DATA = rdata.DATA, d
            try:
                import regress
                return regress.main("new", "base")
            finally:
                rdata.DATA = old

    def test_a_worse_low_mid_band_fails(self):
        # ChatGPT audit 2026-11-18 #8: 150 Hz-2 kHz from -14 to -8 dB read PASS
        self.assertEqual(self.run_pair(summary("base", -15, -14, 1), summary("new", -15, -8, 1)), 1)

    def test_a_worse_treble_hump_fails(self):
        self.assertEqual(self.run_pair(summary("base", -15, -14, 1), summary("new", -15, -14, 5)), 1)

    def test_a_lift_from_too_few_events_is_not_judged(self):
        # 2026-11-23: main's K20A lift (-1.8) was one throttle lift, the new build's (+5.1) the
        # median of three at other moments; matched in time (drive scene) the new one was lower
        ear = lambda sfx, lift, n: (f"k20-{sfx}                 PASS  bank  +14.2<+0.7 ok  wind  -12.9>+29.4 ok  "
                                    f"lift  {lift:+.1f}>+10.4 ok  peak  +22.9>+48.4 ok  lifts {n}\n")
        base = summary("base", -15, -14, 1) + ear("base", -1.8, 1)
        self.assertEqual(self.run_pair(base, summary("new", -15, -14, 1) + ear("new", 5.1, 3)), 0)
        base3 = summary("base", -15, -14, 1) + ear("base", -1.8, 3)
        self.assertEqual(self.run_pair(base3, summary("new", -15, -14, 1) + ear("new", 5.1, 3)), 1)

    def test_the_same_numbers_pass(self):
        self.assertEqual(self.run_pair(summary("base", -15, -14, 1), summary("new", -15, -14, 1)), 0)


if __name__ == "__main__":
    unittest.main()
