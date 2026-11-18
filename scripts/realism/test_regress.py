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

    def test_the_same_numbers_pass(self):
        self.assertEqual(self.run_pair(summary("base", -15, -14, 1), summary("new", -15, -14, 1)), 0)


if __name__ == "__main__":
    unittest.main()
