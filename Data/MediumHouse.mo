within CoSES_Thermal_ProHMo_PHiL.Data;
record MediumHouse "Medium-sized residential house"
  extends BaseBuilding(
    // Zone Areas - Between Small and Big House
    AZone_living=75.0,
    AZone_cellar=60.0,
    AZone_roof=65.0,

    // Zone Heights
    hZone_living=2.6,
    hZone_cellar=2.3,
    hZone_roof=2.5,

    // Initial Temperatures
    TZoneInit_living=293.15,  // 20°C
    TZoneInit_cellar=291.15,  // 18°C
    TZoneInit_roof=290.15,    // 17°C

    // Reference Temperatures
    TRef_living=294.15,  // 21°C
    TRef_cellar=292.15,  // 19°C
    TRef_roof=293.15,    // 20°C

    // Control Parameters
    k_PI=0.28,
    Ti_PI=550,
    yMin_living=0.035,
    yMin_cellar=0.055,
    yMin_roof=0.09,

    // Heating
    cellarHeat=true,
    roofHeat=true,

    // Internal Loads
    P_appliances_living_default=300,
    P_appliances_cellar_default=75,
    P_appliances_roof_default=75,

    // Occupancy - 3-person household
    nPersons_living_default=3,
    nPersons_cellar_default=0,
    nPersons_roof_default=0,

    // Added: Thermal Resistances for Medium House
    Rwall=3.030,         // Good wall insulation (U≈0.33 W/(m²·K) for 1 m²)
    RExtRem=0.135,       // Remaining exterior resistance
    Rfloor=3.571,        // Good floor insulation (U≈0.28 W/(m²·K) for 1 m²)
    RFloorRem=0.135,     // Remaining floor resistance
    Rroof=5.556,         // Excellent roof insulation (U≈0.18 W/(m²·K) for 1 m²)
    RRoofRem=0.135);     // Remaining roof resistance

  annotation(Documentation(info="<html>
<p><b>Medium Residential House Configuration - NEW</b></p>

<p><b>Building Characteristics:</b></p>
<ul>
<li>Living area: 75 m²</li>
<li>Cellar area: 60 m²</li>
<li>Roof area: 65 m²</li>
<li>Typical 3-person household</li>
<li>Between small and large house sizes</li>
</ul>

<li>
Dec 7, 2025, by Karthik Murugesan
</li>
<p><b>Thermal Properties</b></p>
<p>Good residential insulation levels:</p>
<ul>
<li><b>Walls:</b> R = 3.030 K/W (U ≈ 0.33 W/(m²·K)) - Good insulation</li>
<li><b>Floor:</b> R = 3.571 K/W (U ≈ 0.28 W/(m²·K)) - Good insulation</li>
<li><b>Roof:</b> R = 5.556 K/W (U ≈ 0.18 W/(m²·K)) - Excellent insulation</li>
<li><b>Remaining resistances:</b> 0.135 K/W</li>
</ul>

<li>
Oct 30, 2025, by Karthik Murugesan
</li>
<p><b>Energy Performance:</b></p>
<ul>
<li>Suitable for modern family homes</li>
<li>Good energy efficiency</li>
<li>Moderate heating requirements</li>
</ul>
</html>"));
end MediumHouse;
