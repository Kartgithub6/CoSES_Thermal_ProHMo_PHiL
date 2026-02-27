within CoSES_Thermal_ProHMo_PHiL.Data;
record Hospital "Hospital ward"
  extends BaseBuilding(
    // Zone Areas - Hospital layout
    AZone_living=150.0,  // Patient ward
    AZone_cellar=80.0,   // Storage/utilities
    AZone_roof=120.0,    // Additional ward/treatment

    // Zone Heights - Hospital standard
    hZone_living=3.2,
    hZone_cellar=2.8,
    hZone_roof=3.2,

    // Initial Temperatures - Hospital comfort (higher)
    TZoneInit_living=295.15,  // 22°C
    TZoneInit_cellar=291.15,  // 18°C
    TZoneInit_roof=294.15,    // 21°C

    // Reference Temperatures - Strict hospital requirements
    TRef_living=296.15,  // 23°C - Patient comfort
    TRef_cellar=291.15,  // 18°C
    TRef_roof=295.15,    // 22°C

    // Control Parameters - Very tight control for patients
    k_PI=0.40,
    Ti_PI=300,
    yMin_living=0.06,
    yMin_cellar=0.15,
    yMin_roof=0.07,

    // Heating
    cellarHeat=false,
    roofHeat=true,

    // Internal Loads - Medical equipment
    P_appliances_living_default=1500,
    P_appliances_cellar_default=300,
    P_appliances_roof_default=1200,

    // Occupancy - Patients + staff
    nPersons_living_default=20,
    nPersons_cellar_default=0,
    nPersons_roof_default=15,

    // ⭐ NEW: Thermal Resistances for Hospital
    // Hospitals need excellent insulation for energy efficiency and comfort
    Rwall=4.000,         // Excellent wall insulation (U≈0.25 W/(m²·K) for 1 m²)
    RExtRem=0.150,       // Higher remaining resistance for better thermal stability
    Rfloor=4.545,        // Excellent floor insulation (U≈0.22 W/(m²·K) for 1 m²)
    RFloorRem=0.150,     // Higher remaining resistance
    Rroof=7.143,         // Superior roof insulation (U≈0.14 W/(m²·K) for 1 m²)
    RRoofRem=0.150);     // Higher remaining resistance

  annotation(Documentation(info="<html>
<p><b>Hospital Ward Configuration - UPDATED</b></p>

<p><b>Building Characteristics:</b></p>
<ul>
<li>Patient ward: 150 m²</li>
<li>Utilities: 80 m²</li>
<li>Treatment/ward: 120 m²</li>
<li>Higher temperature requirements for patient comfort</li>
<li>Tighter temperature control for medical needs</li>
<li>Medical equipment loads</li>
<li>24/7 operation</li>
</ul>

<p><b>⭐ NEW: Thermal Properties</b></p>
<p>Superior insulation levels for healthcare:</p>
<ul>
<li><b>Walls:</b> R = 4.000 K/W (U ≈ 0.25 W/(m²·K)) - Excellent insulation</li>
<li><b>Floor:</b> R = 4.545 K/W (U ≈ 0.22 W/(m²·K)) - Excellent insulation</li>
<li><b>Roof:</b> R = 7.143 K/W (U ≈ 0.14 W/(m²·K)) - Superior insulation</li>
<li><b>Remaining resistances:</b> 0.150 K/W - Enhanced thermal mass for stability</li>
</ul>

<p><b>Energy Performance:</b></p>
<p>This hospital has superior insulation suitable for:</p>
<ul>
<li>Patient comfort and recovery</li>
<li>Strict temperature control requirements</li>
<li>Continuous operation (24/7)</li>
<li>Energy efficiency despite high comfort standards</li>
<li>Infection control through stable environmental conditions</li>
</ul>
</html>"));
end Hospital;
