within CoSES_Thermal_ProHMo_PHiL.Data;
record Office "Office building"
  extends BaseBuilding(
    // Zone Areas - Office layout
    AZone_living=120.0,  // Open office space
    AZone_cellar=60.0,   // Storage/technical
    AZone_roof=100.0,    // Additional office/meeting rooms

    // Zone Heights - Commercial standard
    hZone_living=3.0,
    hZone_cellar=2.8,
    hZone_roof=3.0,

    // Initial Temperatures - Office comfort
    TZoneInit_living=294.15,  // 21°C
    TZoneInit_cellar=291.15,  // 18°C
    TZoneInit_roof=293.15,    // 20°C

    // Reference Temperatures - Higher for office
    TRef_living=295.15,  // 22°C
    TRef_cellar=291.15,  // 18°C
    TRef_roof=294.15,    // 21°C

    // Control Parameters - Tighter control for office
    k_PI=0.35,
    Ti_PI=400,
    yMin_living=0.05,
    yMin_cellar=0.10,
    yMin_roof=0.06,

    // Heating
    cellarHeat=false,  // Technical room, less heating needed
    roofHeat=true,

    // Internal Loads - Office equipment
    P_appliances_living_default=1000,  // Computers, lighting
    P_appliances_cellar_default=200,   // Server/technical
    P_appliances_roof_default=800,     // Meeting rooms

    // Occupancy - Office hours
    nPersons_living_default=15,
    nPersons_cellar_default=0,
    nPersons_roof_default=10,

    // Added: Thermal Resistances for Office Building
    // Commercial buildings typically have code-compliant insulation
    Rwall=2.500,         // Commercial wall insulation (U≈0.40 W/(m²·K) for 1 m²)
    RExtRem=0.125,       // Remaining exterior resistance
    Rfloor=3.030,        // Commercial floor insulation (U≈0.33 W/(m²·K) for 1 m²)
    RFloorRem=0.125,     // Remaining floor resistance
    Rroof=4.545,         // Good commercial roof insulation (U≈0.22 W/(m²·K) for 1 m²)
    RRoofRem=0.125);     // Remaining roof resistance

  annotation(Documentation(info="<html>
<p><b>Office Building Configuration - UPDATED</b></p>

<p><b>Building Characteristics:</b></p>
<ul>
<li>Main office: 120 m²</li>
<li>Technical/storage: 60 m²</li>
<li>Upper floor: 100 m²</li>
<li>Typical 25-person office</li>
<li>Higher internal loads from office equipment</li>
<li>Increased ceiling height for commercial use</li>
</ul>

<li>
Dec 7, 2025, by Karthik Murugesan
</li>
<p><b>Thermal Properties</b></p>
<p>Commercial-grade insulation levels:</p>
<ul>
<li><b>Walls:</b> R = 2.500 K/W (U ≈ 0.40 W/(m²·K)) - Commercial standard</li>
<li><b>Floor:</b> R = 3.030 K/W (U ≈ 0.33 W/(m²·K)) - Commercial standard</li>
<li><b>Roof:</b> R = 4.545 K/W (U ≈ 0.22 W/(m²·K)) - Good commercial insulation</li>
<li><b>Remaining resistances:</b> 0.125 K/W - Standard commercial thermal mass</li>
</ul>

<li>
Oct 30, 2025, by Karthik Murugesan
</li>
<p><b>Energy Performance:</b></p>
<p>This office building has commercial-grade insulation suitable for:</p>
<ul>
<li>Meeting commercial building codes</li>
<li>Balanced with high internal heat gains from equipment and occupants</li>
<li>Cost-effective operation during business hours</li>
<li>Moderate cooling loads in summer due to internal gains</li>
</ul>
</html>"));
end Office;
