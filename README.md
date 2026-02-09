# Stormworks Lua Ballistics

Production ballistic scripts for Stormworks microcontrollers.

## Layout

- `src/ballistics-computer-3d.lua`: Primary ballistic calculator (active).
- `src/dive-bomb-assist.lua`: Dive-bomb aiming and release assist script.
- `src/target-tracker.lua`: Radar-to-world target tracker with parallax compensation.
- `src/deprecated/`: Deprecated and archived ballistics experiments/utilities.

## Notes

`ballistics-computer-3d.lua` and `dive-bomb-assist.lua` are the active scripts for this repository. Other historical scripts were moved into `src/deprecated/`.
