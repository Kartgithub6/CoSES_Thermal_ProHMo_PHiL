within CoSES_Thermal_ProHMo_PHiL.Data;
record Mall "Shopping mall/retail center"
  extends BaseBuilding(
    // Zone Areas - Mall layout
    AZone_living=300.0,  // Main retail floor
    AZone_cellar=180.0,  // Storage/loading
    AZone_roof=250.0,    // Additional retail/food court

    // Zone Heights - Commercial retail
    hZone_living=4.5,  // High ceilings for retail
    hZone_cellar=3.5,  // Loading/storage
    hZone_roof=4.0,    // Food court/retail

    // Initial Temperatures - Retail comfort
    TZoneInit_living=294.15,  // 21°C
    TZoneInit_cellar=288.15,  // 15°C
    TZoneInit_roof=293.15,    // 20°C

    // Reference Temperatures - Shopping comfort
    TRef_living=295.15,  // 22°C - Main floor
    TRef_cellar=288.15,  // 15°C - Storage
    TRef_roof=294.15,    // 21°C - Food court

    // Control Parameters - Moderate for large spaces
    k_PI=0.22,
    Ti_PI=650,
    yMin_living=0.04,
    yMin_cellar=0.02,
    yMin_roof=0.05,

    // Heating
    cellarHeat=false,
    roofHeat=true,

    // Internal Loads - Retail lighting and equipment
    P_appliances_living_default=8000,   // Extensive retail lighting
    P_appliances_cellar_default=1000,   // Loading equipment
    P_appliances_roof_default=6000,     // Food court + retail

    // Occupancy - Customers + staff (highly variable)
    nPersons_living_default=100,
    nPersons_cellar_default=10,
    nPersons_roof_default=80,

    // ⭐ NEW: Thermal Resistances for Mall
    // Malls have moderate insulation balanced with high internal gains
    Rwall=2.500,         // Commercial wall insulation (U≈0.40 W/(m²·K) for 1 m²)
    RExtRem=0.120,       // Remaining exterior resistance
    Rfloor=3.030,        // Commercial floor insulation (U≈0.33 W/(m²·K) for 1 m²)
    RFloorRem=0.120,     // Remaining floor resistance
    Rroof=4.545,         // Good commercial roof (U≈0.22 W/(m²·K) for 1 m²)
    RRoofRem=0.120);     // Remaining roof resistance

  annotation(Documentation(info="<html>
<p><b>Shopping Mall/Retail Center - NEW</b></p>

<p><b>Building Characteristics:</b></p>
<ul>
<li>Main retail: 300 m²</li>
<li>Storage/loading: 180 m²</li>
<li>Food court/retail: 250 m²</li>
<li>Very high occupancy</li>
<li>Extended operating hours</li>
<li>High internal heat gains from lighting and people</li>
</ul>

<p><b>⭐ Thermal Properties</b></p>
<p>Commercial retail insulation:</p>
<ul>
<li><b>Walls:</b> R = 2.500 K/W (U ≈ 0.40 W/(m²·K)) - Commercial standard</li>
<li><b>Floor:</b> R = 3.030 K/W (U ≈ 0.33 W/(m²·K)) - Commercial standard</li>
<li><b>Roof:</b> R = 4.545 K/W (U ≈ 0.22 W/(m²·K)) - Good commercial</li>
</ul>

<p><b>Energy Performance:</b></p>
<ul>
<li>Balance insulation with high internal gains</li>
<li>Large spaces with variable loads</li>
<li>Cooling often more critical than heating</li>
</ul>
</html>"));
end Mall;
