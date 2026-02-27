within CoSES_Thermal_ProHMo_PHiL.Data;
record BigHouse "Large residential house"
  extends BaseBuilding(
    // Zone Areas - Larger
    AZone_living=100.0,
    AZone_cellar=80.0,
    AZone_roof=90.0,

    // Zone Heights
    hZone_living=2.8,
    hZone_cellar=2.5,
    hZone_roof=2.6,

    // Initial Temperatures
    TZoneInit_living=293.15,
    TZoneInit_cellar=291.15,
    TZoneInit_roof=290.15,

    // Reference Temperatures
    TRef_living=294.15,
    TRef_cellar=292.15,
    TRef_roof=293.15,

    // Control Parameters - Different tuning for larger spaces
    k_PI=0.25,
    Ti_PI=600,
    yMin_living=0.04,
    yMin_cellar=0.06,
    yMin_roof=0.10,

    // Heating
    cellarHeat=true,
    roofHeat=true,

    // Internal Loads - Higher
    P_appliances_living_default=400,
    P_appliances_cellar_default=100,
    P_appliances_roof_default=100,

    // Occupancy - Family house
    nPersons_living_default=4,
    nPersons_cellar_default=0,
    nPersons_roof_default=1,

    // ⭐ NEW: Thermal Resistances for Big House
    // Large houses often have better insulation due to newer construction
    Rwall=3.333,         // Good wall insulation (U≈0.30 W/(m²·K) for 1 m²)
    RExtRem=0.140,       // Remaining exterior resistance (better thermal mass)
    Rfloor=4.000,        // Good floor insulation (U≈0.25 W/(m²·K) for 1 m²)
    RFloorRem=0.140,     // Remaining floor resistance
    Rroof=6.250,         // Excellent roof insulation (U≈0.16 W/(m²·K) for 1 m²)
    RRoofRem=0.140);     // Remaining roof resistance

  annotation(Documentation(info="<html>
<p><b>Large Residential House Configuration - UPDATED</b></p>

<p><b>Building Characteristics:</b></p>
<ul>
<li>Living area: 100 m²</li>
<li>Cellar area: 80 m²</li>
<li>Roof area: 90 m²</li>
<li>Typical 4-5 person household</li>
<li>Higher ceiling heights for comfort</li>
</ul>

<p><b>⭐ NEW: Thermal Properties</b></p>
<p>Better-than-average insulation levels:</p>
<ul>
<li><b>Walls:</b> R = 3.333 K/W (U ≈ 0.30 W/(m²·K)) - Good insulation</li>
<li><b>Floor:</b> R = 4.000 K/W (U ≈ 0.25 W/(m²·K)) - Good insulation</li>
<li><b>Roof:</b> R = 6.250 K/W (U ≈ 0.16 W/(m²·K)) - Excellent insulation</li>
<li><b>Remaining resistances:</b> 0.140 K/W - Higher thermal mass</li>
</ul>

<p><b>Energy Performance:</b></p>
<p>This large house has improved insulation suitable for:</p>
<ul>
<li>Modern residential standards</li>
<li>Energy-efficient operation</li>
<li>Lower heating costs despite larger size</li>
<li>Comfortable year-round living</li>
</ul>
</html>"));
end BigHouse;
