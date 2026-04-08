within CoSES_Thermal_ProHMo_PHiL.Data;
record School "School building"
  extends BaseBuilding(
    // Zone Areas - School layout
    AZone_living=140.0,  // Main classroom
    AZone_cellar=100.0,  // Gym/multipurpose
    AZone_roof=120.0,    // Additional classrooms

    // Zone Heights
    hZone_living=3.5,  // High ceilings for classroom
    hZone_cellar=4.0,  // High for gym
    hZone_roof=3.5,

    // Initial Temperatures
    TZoneInit_living=293.15,
    TZoneInit_cellar=291.15,
    TZoneInit_roof=293.15,

    // Reference Temperatures
    TRef_living=294.15,  // 21°C
    TRef_cellar=291.15,  // 18°C (gym can be cooler)
    TRef_roof=294.15,

    // Control Parameters
    k_PI=0.28,
    Ti_PI=550,
    yMin_living=0.04,
    yMin_cellar=0.08,
    yMin_roof=0.05,

    // Heating
    cellarHeat=true,
    roofHeat=true,

    // Internal Loads - Lighting + equipment
    P_appliances_living_default=800,
    P_appliances_cellar_default=400,
    P_appliances_roof_default=700,

    // Occupancy - Students + teachers
    nPersons_living_default=30,
    nPersons_cellar_default=25,
    nPersons_roof_default=30,

    // Added: Thermal Resistances for School
    // Schools need good insulation for energy efficiency and comfort
    Rwall=3.030,         // Good wall insulation (U≈0.33 W/(m²·K) for 1 m²)
    RExtRem=0.135,       // Remaining exterior resistance
    Rfloor=3.571,        // Good floor insulation (U≈0.28 W/(m²·K) for 1 m²)
    RFloorRem=0.135,     // Remaining floor resistance
    Rroof=5.556,         // Excellent roof insulation (U≈0.18 W/(m²·K) for 1 m²)
    RRoofRem=0.135);     // Remaining roof resistance

  annotation(Documentation(info="<html>
<p><b>School Building Configuration - UPDATED</b></p>

<p><b>Building Characteristics:</b></p>
<ul>
<li>Main classroom: 140 m²</li>
<li>Gym/multipurpose: 100 m²</li>
<li>Additional classrooms: 120 m²</li>
<li>High occupancy (up to 85 people)</li>
<li>Variable internal loads</li>
<li>High ceilings for ventilation and comfort</li>
</ul>

<li>
Dec 7, 2025, by Karthik Murugesan
</li>
<p><b>Thermal Properties</b></p>
<p>Educational-grade insulation levels:</p>
<ul>
<li><b>Walls:</b> R = 3.030 K/W (U ≈ 0.33 W/(m²·K)) - Good insulation</li>
<li><b>Floor:</b> R = 3.571 K/W (U ≈ 0.28 W/(m²·K)) - Good insulation</li>
<li><b>Roof:</b> R = 5.556 K/W (U ≈ 0.18 W/(m²·K)) - Excellent insulation</li>
<li><b>Remaining resistances:</b> 0.135 K/W - Good thermal mass</li>
</ul>

<li>
Oct 30, 2025, by Karthik Murugesan
</li>
<p><b>Energy Performance:</b></p>
<p>This school has good insulation suitable for:</p>
<ul>
<li>Learning environment comfort</li>
<li>High occupancy loads (heat from students)</li>
<li>Daytime operation with nighttime setback</li>
<li>Cost-effective operation for public buildings</li>
<li>Meets educational building standards</li>
</ul>
</html>"));
end School;
