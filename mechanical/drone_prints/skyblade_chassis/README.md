# Skyblade Drone Chassis G-code

This folder contains the first-pass slice outputs for the Skyblade v2.0 modular drone frame test chassis.

## Printer folders
- `athorbot/` — Athorbot 0.6 nozzle slice
- `prusa/` — Prusa 0.6 nozzle slice

## Slice notes
- Mesh source: `3dprint/Skyblade v2.0 Modular Drone Frame - 2517368/files/Skyblade_Main_Chassis_Test_Print.stl`
- Layer height: 0.28 mm
- Perimeters: 3
- Top layers: 5
- Bottom layers: 4
- Infill: 15%
- PLA baseline slice from PrusaSlicer
- Athorbot extruder should be calibrated in firmware to 397.5 E-steps/mm; slicer flow is left at 1.0
- No Blender pass used; the mesh was already manifold.
