within CoSES_Thermal_ProHMo_PHiL.Data;
record Factory "Factory/manufacturing facility"
  extends BaseBuilding(
    // Zone Areas - Factory layout
    AZone_living=200.0,  // Main production floor
    AZone_cellar=150.0,  // Storage/warehouse
    AZone_roof=180.0,    // Office/assembly area

    // Zone Heights - High ceilings for industrial use
    hZone_living=5.0,
    hZone_cellar=4.5,
    hZone_roof=4.0,

    // Initial Temperatures - Lower for industrial
    TZoneInit_living=291.15,  // 18°C
    TZoneInit_cellar=288.15,  // 15°C
    TZoneInit_roof=291.15,    // 18°C

    // Reference Temperatures - Industrial comfort
    TRef_living=292.15,  // 19°C - Workers generate heat
    TRef_cellar=288.15,  // 15°C - Storage area
    TRef_roof=293.15,    // 20°C - Office area

    // Control Parameters - Less tight for industrial
    k_PI=0.20,
    Ti_PI=700,
    yMin_living=0.03,
    yMin_cellar=0.02,
    yMin_roof=0.04,

    // Heating
    cellarHeat=false,  // Minimal heating in storage
    roofHeat=true,

    // Internal Loads - Heavy machinery and equipment
    P_appliances_living_default=5000,  // Production equipment
    P_appliances_cellar_default=500,   // Storage systems
    P_appliances_roof_default=1500,    // Office + light assembly

    // Occupancy - Workers
    nPersons_living_default=40,
    nPersons_cellar_default=5,
    nPersons_roof_default=20,

    // ⭐ NEW: Thermal Resistances for Factory
    // Factories have basic insulation - cost-driven
    Rwall=2.000,         // Basic wall insulation (U≈0.50 W/(m²·K) for 1 m²)
    RExtRem=0.110,       // Remaining exterior resistance
    Rfloor=2.500,        // Basic floor insulation (U≈0.40 W/(m²·K) for 1 m²)
    RFloorRem=0.110,     // Remaining floor resistance
    Rroof=3.571,         // Moderate roof insulation (U≈0.28 W/(m²·K) for 1 m²)
    RRoofRem=0.110);     // Remaining roof resistance

  annotation(Documentation(info="<html>
<p><b>Factory/Manufacturing Facility - NEW</b></p>

<p><b>Building Characteristics:</b></p>
<ul>
<li>Production floor: 200 m²</li>
<li>Storage/warehouse: 150 m²</li>
<li>Office/assembly: 180 m²</li>
<li>High ceilings for equipment and ventilation</li>
<li>High internal heat gains from machinery</li>
<li>Lower temperature requirements</li>
</ul>

<p><b>⭐ Thermal Properties</b></p>
<p>Basic industrial insulation:</p>
<ul>
<li><b>Walls:</b> R = 2.000 K/W (U ≈ 0.50 W/(m²·K)) - Basic insulation</li>
<li><b>Floor:</b> R = 2.500 K/W (U ≈ 0.40 W/(m²·K)) - Basic insulation</li>
<li><b>Roof:</b> R = 3.571 K/W (U ≈ 0.28 W/(m²·K)) - Moderate insulation</li>
</ul>

<p><b>Energy Performance:</b></p>
<ul>
<li>Cost-effective construction</li>
<li>High internal heat gains offset heating needs</li>
<li>Focus on production over comfort</li>
</ul>
</html>"));
end Factory;
