#!/usr/bin/env python3
"""Each noise source of a preset alone, at its absolute level, next to the measured anchors.

Renders `--render held:RPM:LOAD:3` with every `sound.*` source off but one (the exhaust's
pulses cannot be switched off: "exhaust" is pulses + valve jets + tail jets, and every other
row still carries the pulses — read a source's row where it rises above "exhaust"), and prints
dB Lin / dB(A) at the listener (3 m) and converted to the anchors' distances by spherical
spreading: structure to 1 m (+9.5 dB), exhaust orifice to 0.25 m (+21.6 dB).

Anchors (docs/superpowers/specs/2026-11-05-measured-source-levels-design.md): structure at
1 m, petrol, full load 88–92 dB(A) at 3000 rpm, 98–105 at 5000–6000 (Lalor 1980 SAE 800402
Fig. 7; Raff & Perry 1973 Fig. 1); exhaust orifice at 0.25 m 104–110 dB Lin (Davies & Holland
1999 Fig. 1c).

usage: source_levels.py BINARY PRESET.yaml RPM LOAD [OUTDIR]
       source_levels.py --self-test
"""
import math, os, re, subprocess, sys, tempfile, wave
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import levels as L  # noqa: E402
import modulation as M  # noqa: E402

SOURCES = ["flow_noise", "intake", "valve_jet", "duct_flow", "block", "mechanical",
           "chamber_ring", "main_bearings", "piston_slap"]
ALONE = {
    "exhaust": ["flow_noise", "valve_jet", "chamber_ring"],
    "intake": ["intake"],
    "block": ["block"],
    "mechanical": ["mechanical", "main_bearings", "piston_slap"],
    "all": SOURCES,
}
TO_1M = 20 * math.log10(3.0 / 1.0)
TO_025M = 20 * math.log10(3.0 / 0.25)


def with_sources(text, on):
    """The preset with each `sound.*` source it has at 1.0 if in `on`, else 0.0 (a source the
    binary does not know is not in its saved file, and is left out)."""
    for k in SOURCES:
        v = "1.0" if k in on else "0.0"
        text = re.sub(rf"(\n    {k}: )[0-9.]+", rf"\g<1>{v}", text)
    return text


def self_test():
    fs = M.FS
    t = np.arange(fs * 2) / fs
    x = math.sqrt(2) * np.sin(2 * np.pi * 1000 * t) / L.FULL_SCALE_PA  # 1 Pa rms
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "s.wav")
        with wave.open(p, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(fs)
            w.writeframes((x * 32767).astype("<i2").tobytes())
        lin, a, _ = L.levels(M.load(p))
    assert abs(lin - 94.0) < 0.1, f"Lin {lin:.2f}"
    assert abs(a - 94.0) < 0.2, f"A {a:.2f}"
    print("self-test ok")


def main(argv):
    if argv[:1] == ["--self-test"]:
        return self_test()
    binary, preset, rpm, load = argv[0], argv[1], int(argv[2]), float(argv[3])
    out = argv[4] if len(argv) > 4 else tempfile.mkdtemp()
    # the preset as this binary saves it: every source it knows is in the file
    saved = os.path.join(out, "saved.yaml")
    subprocess.run([binary, "--engine", preset, "--save", saved], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    text = open(saved).read()
    print(f"{os.path.basename(preset)} {rpm} rpm load {load} (listener 3 m)")
    for name, on in ALONE.items():
        yml = os.path.join(out, f"{name}_{rpm}_{load}.yaml")
        wav = yml[:-5] + ".wav"
        with open(yml, "w") as f:
            f.write(with_sources(text, on))
        subprocess.run([binary, "--engine", yml, "--render", f"held:{rpm}:{load}:3",
                        "--out", wav], check=True, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        lin, a, _ = L.levels(M.load(wav)[M.FS // 2:])
        print(f"  {name:10s} {lin:6.1f} dB {a:6.1f} dB(A) | at 1 m {a + TO_1M:6.1f} dB(A)"
              f" | at 0.25 m {lin + TO_025M:6.1f} dB")


if __name__ == "__main__":
    main(sys.argv[1:])
