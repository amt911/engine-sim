# engine-sim

Real-time engine sound simulator in Rust: build an engine visually, watch it run, hear the physics.

```bash
cargo run -p engine-app --release                                  # opens the app
cargo run -p engine-app --release -- --engine presets/lfa-v10.yaml # load an engine file
```

Keys: **W** / Up = full throttle, **S** = starter. Edit the engine in the left panel; save or load
it as YAML/JSON (or drop a file on the window).

- Wishes: [`docs/specs.md`](docs/specs.md) · Design: [`docs/superpowers/specs/`](docs/superpowers/specs/)
- Research (physics, DSP, papers): [`docs/research/simulador-motor-rust.md`](docs/research/simulador-motor-rust.md)
- Progress tags: [`CHANGELOG.md`](CHANGELOG.md)
- Agents: [`AGENTS.md`](AGENTS.md)

Linux deps: `alsa-lib` (Arch) / `libasound2-dev` (Debian/Ubuntu).

## References

Every physical model and constant in the code cites its source in a doc comment; the formulas are
tested against their references in [`docs/physics-validation.md`](docs/physics-validation.md).
The main works used:

**Engine and combustion**
- J. B. Heywood, *Internal Combustion Engine Fundamentals*, McGraw-Hill, 1988 (orifice flow, NASA
  gas properties, heating values, efficiencies).
- J. I. Ghojel, "Review of the development and applications of the Wiebe function", *Int. J. Engine
  Research* 11(4), 2010 (burn fraction); Ferguson & Kirkpatrick, *Internal Combustion Engines*.
- N. D. Watson, A. D. Pilley, M. Marzouk, SAE 800029 (diesel premixed and diffusion burn).
- M. Metghalchi, J. C. Keck, *Combustion and Flame* 48, 1982 (laminar flame speed).
- N. Ozdor, M. Dulger, E. Sher, 1994 (cycle-to-cycle variability).
- G. Woschni, SAE 670931 (in-cylinder heat transfer); Dittus–Boelter (pipe heat transfer).
- D. Sandoval, J. B. Heywood, SAE 2003-01-0725 (friction by components).
- G. P. Blair, *Design and Simulation of Four-Stroke Engines*, SAE R-186, 1999.
- L. Guzzella, C. Onder, *Introduction to Modeling and Control of IC Engine Systems*, Springer
  2010; L. Eriksson, L. Nielsen, *Modeling and Control of Engines and Drivelines*, Wiley 2014
  (driveline, clutch, vehicle); E. Hendricks, SAE 890564 (mean-value models).
- Bosch, *Gasoline Engine Management* (ECU strategies: idle, cut-offs, torque intervention).

**Turbocharging**
- N. Watson, M. S. Janota, *Turbocharging the Internal Combustion Engine*, Macmillan 1982.
- S. L. Dixon, C. A. Hall, *Fluid Mechanics and Thermodynamics of Turbomachinery*, ch. 7.
- Sriganesh, Dehner, Selamet, *Noise Control Eng. J.* 70(2), 2022 (turbocharger noise).

**Gas dynamics and numerics**
- E. F. Toro, *Riemann Solvers and Numerical Methods for Fluid Dynamics*, 3rd ed., Springer 2009
  (HLLC, shock-tube tests).
- R. Borges, M. Carmona, B. Costa, W. S. Don, *J. Comput. Phys.* 227, 2008 (WENO5-Z);
  C.-W. Shu, S. Osher, *J. Comput. Phys.* 77, 1988 (SSP-RK3).
- J. D. Anderson, *Modern Compressible Flow*; R. S. Benson, *The Thermodynamics and Gas Dynamics of
  Internal-Combustion Engines*, vol. 1, OUP 1982 (junction and valve boundaries).
- D. E. Winterbone, R. J. Pearson, *Theory of Engine Manifold Design*, PEP 2000.
- M. Msallam et al., *Acta Acustica* 86, 2000 (nonlinear wave steepening); F. Tasker et al. 2008.
- I. E. Idelchik, *Handbook of Hydraulic Resistance* (pressure-loss coefficients);
  R. K. Shah, A. L. London, *Laminar Flow Forced Convection in Ducts*, 1978.
- R. P. Brent, *Algorithms for Minimization without Derivatives*, 1973 (root finding).

**Duct acoustics and radiation**
- H. Levine, J. Schwinger, "On the radiation of sound from an unflanged circular pipe",
  *Phys. Rev.* 73, 1948; F. Silva, P. Guillemain, J. Kergomard, B. Mallaroni, A. N. Norris,
  *J. Sound Vib.* 322, 2009 (open-end reflection).
- M. L. Munjal, *Acoustics of Ducts and Mufflers*, 2nd ed., Wiley 2014; L. E. Kinsler et al.,
  *Fundamentals of Acoustics* (silencers, resonators, wall losses).
- R. M. Munt, 1977; S. W. Rienstra (open end with mean flow); W. Ronneberger, 1967; U. Ingard,
  H. Ising, 1967 (orifice nonlinearity).
- M. E. Delany, E. N. Bazley, *Applied Acoustics* 3, 1970 (porous materials).
- P. O. A. L. Davies, K. R. Holland, *J. Sound Vib.* 223(3), 1999 (measured exhaust noise).
- SAE ARP876 (jet noise); M. J. Lighthill (eighth-power law); Hoch et al., 1973;
  Golebiowski & Seiner, AIAA 96-1716 (jet spectra).
- ISO 9613-2 (ground and wall reflection).

**Engine structure noise**
- T. Priede; M. Anderton, SAE 790270 (combustion noise through the block).
- N. Lalor, E. C. Grover, T. Priede, SAE 800402, 1980; S. D. Haddad, H. L. Pullen, 1974 (piston
  slap and mechanical noise); Zheng et al., 2020 (valvetrain impacts).
- J. A. Raff, R. D. H. Perry, *J. Sound Vib.* 28(3), 1973 (intake noise).

**Audio DSP**
- J. O. Smith, *Physical Audio Signal Processing*, CCRMA (waveguides, FDN reverberation).
- T. I. Laakso et al., *IEEE Signal Processing Magazine*, 1996 (fractional delay).
- J.-M. Jot, A. Chaigne, AES 1991 (feedback-delay-network reverberation).
- A. Simper, "Linear trapezoidal integrated SVF", Cytomic, 2013 (filters).
- R. Bencina, "Real-time audio programming 101" (real-time audio rules).
- S. Vigna, 2016 (xoshiro/xorshift random numbers).
