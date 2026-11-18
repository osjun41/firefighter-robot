# Bill of Materials (BOM)

This directory contains the public hardware Bill of Materials for the firefighter robot project.

## Files

- `ASMR2026-BOM_public.xlsx` — Public Excel workbook containing the main BOM and actuator sub-BOMs.
- `ASMR2026-BOM.csv` — Main robot BOM in a GitHub-readable CSV format.
- `Actuator-M6C12-BOM.csv` — BOM for the M6C12 actuator.
- `Actuator-5010-BOM.csv` — BOM for the 5010 actuator.

## BOM structure

The main BOM covers the computing platform, actuators, bearings, batteries and power components, CAN interface, controller, connectors, fasteners, wiring, and structural components. The actuator BOMs provide additional details for bearings, fasteners, motors, structural material, and motor controllers.

## Notes

- Quantities and prices reflect the project BOM at the time this file was prepared and may change.
- Supplier links are included where available for reproducibility.
- Some entries are intentionally left blank where the exact quantity, price, or supplier has not yet been finalized.
- Internal purchasing records, including order dates and purchase status, are intentionally excluded from this public version.
- For the latest design state, refer to the project's Onshape document and the CAD release information in `/cad`.

## Recommended repository structure

```text
firefighter-robot/
├── cad/
│   ├── robot/
│   └── README.md
├── bom/
│   ├── ASMR2026-BOM_public.xlsx
│   ├── ASMR2026-BOM.csv
│   ├── Actuator-M6C12-BOM.csv
│   ├── Actuator-5010-BOM.csv
│   └── README.md
└── README.md
```
