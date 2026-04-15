# Thermo-Energetic Analysis of Multi-Zone Buildings using Digital Twin Power Hardware-in-the-Loop Environments

**CoSES_Thermal_ProHMo_PHiL**

Master's Thesis — Karthik Murugesan  
Chair of Renewable and Sustainable Energy Systems, Technical University of Munich  
Supervisor: Ulrich Ganslmeier M.Sc. | Advisor: Prof. Dr. rer. nat. Thomas Hamacher  
Submitted: March 31, 2026

---

## Overview

This repository contains the complete source code for a multi-zone residential building thermal digital twin developed for Power Hardware-in-the-Loop (PHiL) experiments at the [CoSES Smart Grid Laboratory](https://www.mep.tum.de/en/mep/coses/), TU Munich (Garching).

The core model, **ThreeZoneBuilding_PHiL**, represents a three-zone residential building (living area, cellar, roof office) connected to a shared hydronic heating circuit. It is validated across three simulation platforms — Dymola, NI VeriStand (real-time PHiL), and SimulationX — and exported as an FMI 2.0 Co-Simulation FMU for hardware deployment.

### Key Contributions

| # | Contribution |
|---|---|
| 1 | Multi-zone Modelica building model (`ThreeZoneBuilding_PHiL`) using IBPSA FourElements RC network |
| 2 | Zone-level hydronic heating with EN 442-2 radiators and PI temperature controllers |
| 3 | FMU export (FMI 2.0, Co-Simulation, 32-bit) and real-time deployment in NI VeriStand 2018 |
| 4 | Custom C wrapper DLL (`fmu_veristand_wrapper.c`) with SEH auto-recovery for VeriStand integration |
| 5 | Cross-platform validation against CoSES SimulationX SF1 reference model |
| 6 | Python/Flask automation framework (`CoSES Project Builder`) for generating Dymola simulation variants |

---

## Repository Structure

```
CoSES_Thermal_ProHMo_PHiL/
│
├── package.mo                        # Root Modelica package (load this in Dymola)
├── package.order                     # Package load order
│
├── BuildingSystem/                   # Core thermal zone and building models
│   └── Building/
│       └── HeatedZone.mo             # Single-zone IBPSA FourElements RC model (basic building model)
│
├── ThermoEnergeticAnalysis/          # Post-processing models and energy balance tools
│   ├── ThreeZoneBuilding_optimized.mo   # Basic reorganized building model of HeatedZone.mo
│   └── ThreeZoneBuilding_PHiL_optimized_withWeather.mo  # PHiL-ready model (FMU source)
│ 
│
├── HydronicSystem/                   # Hydronic heating sub-system
│   └── SystemWithZoneAndHydronics_Optimized.mo  # Radiator + valve + PI controller per zone
│
├── Examples/                         # Simulation test harnesses
│   ├── TestThreeZoneBuilding_withWeather.mo      # Scenario A (dynamic 24-hour)
│   └── TestThreeZoneBuilding_StaticInputs.mo     # Scenario B (VeriStand-aligned static)
│ 
├── Data/                             # Simulation input data
│   ├── SimulationData/building/inner_loads/zone_x.txt  # Occupancy/load profiles
│   └── BaseBuilding, SmallHouse, MediumHouse, LargeHouse, Office, Hospital, School, Factory, Hotel, Mall # Record for different building parameters
│
├── Interfaces/                       # Modelica connector definitions
│   └── EnvironmentConditions         # All environment properties
│
├── HeatPorts/                        # Thermal connector definitions
│   ├── WaterToHeatPort               # Non-intrusive adapter between hydronic loop and HeatPort
│   └── WaterToHeatPort_SimpleWater   # Non-intrusive adapter between hydronic loop and HeatPort - SimpleWater version
│ 
├── HydronicSystem/
│   ├── SystemWithZoneAndHydronics    # Hydronic loop segment + thermal port for zone connection
│   ├── SystemWithZoneAndHydronics_SimpleWater # Hydronic loop segment + thermal port for zone connection - SimpleWater version
│   └── SystemWithZoneAndHydronics_optimized  # Hydronic loop with EN442-2 radiator - Final Version
│ 
├── HeatGenerators/
│   ├── SimpleCHP                     # Simplified CHP (based on SimX physics)
│   ├── SimpleCondensingBoiler        # Simplified Condensing Boiler model for Dymola
│   ├── SimpleCHP v2                  # Simplified CHP - debugged version
│   └── SimpleSolarThermal            # Simplified Solar Thermal Collector Model for Dymola
│
├── BoundaryConditions/               # Weather and disturbance signal wrappers
│   ├── Bus                           # Data bus that stores weather data
│   ├── ReaderTMY3                    # Reader for TMY3 weather data
│
├── IBPSA                             # Library with models for building energy and control systems (For FourElements.HeatedZone.mo)
│ 
├── Python Builder/                   # Flask-based automation framework
│   ├── CoSES_Thermal_ProHMo_PHiL.py  # Main Flask server (localhost:5000)
│   ├── CoSES_Complete_Workflow.py    # HTML UI template and configuration interface
│   └── perfect_generator.py         # PerfectDymolaVariantGenerator class
│
├── Plots/                            # Output plot scripts and generated figures
├── Backup/                           # Model version backups
├── Building_CHP_CB_PHiL.bak.mo       # Archived earlier model variant
├── ConvertFromCoSES_Thermal_ProHMo_PHiL.mos  # Dymola conversion script
│
├── blob/main/data/
│   └── veristand_model_export.pdf    # Step-by-step VeriStand FMU import guide
│
├── dsmodel.c                         # Auto-generated Dymola C model code
├── dsfinal.txt                       # Final simulation states (Dymola)
└── buildlog.txt                      # Dymola build log

```
---

## Python Automation Framework (CoSES Project Builder)

The Flask-based tool automates Dymola project variant generation. It eliminates manual `.mo` file editing by applying regex substitution to inject user-specified parameters.

### Starting the Server

```bash
cd "Python Builder"
pip install flask scipy matplotlib numpy
python CoSES_Thermal_ProHMo_PHiL.py
# Open browser: http://localhost:5000
```

### Workflow

1. **Select building type** (9 presets from Small House 70 m² to Mall 500 m²) or use Custom
2. **Set project name**, simulation duration, and weather location
3. Click **Step 1: Generate Project** → creates timestamped Dymola project folder and launches Dymola
4. Press **F9** in Dymola to simulate → writes `.mat` result file
5. Click **Step 2: Generate 9 Plots** → reads `.mat` and renders all output figures in browser

### Building Presets

| Building Type | Living [m²] | Cellar [m²] | Roof [m²] | kP | Ti [s] |
|---------------|-------------|-------------|-----------|-----|--------|
| Small House | 70 | 50 | 40 | 0.35 | 400 |
| Medium House | 100 | 80 | 60 | 0.30 | 500 |
| Large House | 130 | 100 | 80 | 0.25 | 600 |
| Office | 150 | 100 | 80 | 0.25 | 600 |
| School | 200 | 120 | 100 | 0.20 | 700 |
| Factory | 250 | 150 | 120 | 0.20 | 700 |
| Hotel | 300 | 200 | 150 | 0.18 | 800 |
| Hospital | 400 | 150 | 200 | 0.18 | 800 |
| Mall | 500 | 300 | 250 | 0.15 | 900 |

### Key Known Issues Fixed

- `buildingData` parameter not updating: required a `redeclare` regex branch for `Examples/` files
- Dynamic package prefix extraction: the generator now reads the prefix from `package.mo` rather than hardcoding it

---

## Simulation Scenarios

### Scenario A — Dynamic 24-Hour (Dymola)

Uses CombiTimeTable input blocks with IBPSA Munich weather data (`DEU_Munich.108660_IWEC.mos`), occupancy schedules (peak 3 persons living room at 21:00), and appliance profiles (750 W living room 18:00–23:00).

- Solver: DASSL | Tolerance: 1e-6 | Step output: 60 s
- Supply temperature: 45–50 °C (morning boost 08:00–18:00)

### Scenario B — Static Inputs (VeriStand-aligned, Dymola)

All CombiTimeTable blocks replaced by `Modelica.Blocks.Sources.Constant` blocks at values matching the VeriStand workspace (Table 5.4 in thesis). Used for direct platform comparison.

---

## Repository

**GitHub**: https://github.com/Kartgithub6/CoSES_Thermal_ProHMo_PHiL

**VeriStand export guide**: `blob/main/data/veristand_model_export.pdf`

---

## Citation

If you use this model or framework, please cite:

> Murugesan, K. (2026). *Thermo-Energetic Analysis of Multi-Zone Buildings using Digital Twin Power Hardware-in-the-Loop Environments*. Master's Thesis, Chair of Renewable and Sustainable Energy Systems, Technical University of Munich.

---

## License

This repository is shared for research and instructional purposes at the Chair of Renewable and Sustainable Energy Systems, TUM, per the Declaration for Transfer of Thesis. Copyright and personal right of use remain with the author.
