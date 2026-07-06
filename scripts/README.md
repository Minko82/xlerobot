# Utility scripts

- **`calc_power.py`** — power-budget model for the Tri-Bus topology. Computes
  per-bus current draw from motor load, acceleration settings, and motor count,
  using coefficients from empirical lab testing. Run `python scripts/calc_power.py`.

Motor ID assignment lives at the repository root (`set_motor_id.py`) because the
[Build Guide](https://minko82.github.io/xlerobot-pro-website/build-guide.html)
references it there.
