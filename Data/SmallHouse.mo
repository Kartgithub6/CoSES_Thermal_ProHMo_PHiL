within CoSES_Thermal_ProHMo_PHiL.Data;
record SmallHouse "Small residential house"
  extends BaseBuilding(
    // Zone Areas
    AZone_living=50.0,
    AZone_cellar=40.0,
    AZone_roof=45.0,

    // Zone Heights
    hZone_living=2.5,
    hZone_cellar=2.2,
    hZone_roof=2.3,

    // Initial Temperatures (20°C living, 18°C cellar, 17°C roof)
    TZoneInit_living=293.15,
    TZoneInit_cellar=291.15,
    TZoneInit_roof=290.15,

    // Reference Temperatures (21°C living, 19°C cellar, 20°C roof)
    TRef_living=294.15,
    TRef_cellar=292.15,
    TRef_roof=293.15,

    // Control Parameters
    k_PI=0.3,
    Ti_PI=500,
    yMin_living=0.03,
    yMin_cellar=0.05,
    yMin_roof=0.08,

    // Heating
    cellarHeat=true,
    roofHeat=true,

    // Internal Loads
    P_appliances_living_default=200,
    P_appliances_cellar_default=50,
    P_appliances_roof_default=50,

    // Occupancy
    nPersons_living_default=2,
    nPersons_cellar_default=0,
    nPersons_roof_default=0,

    // Added: Thermal Resistances for Small House
    // Small houses typically have standard insulation
    Rwall=2.857,         // Standard wall insulation (U≈0.35 W/(m²·K) for 1 m²)
    RExtRem=0.130,       // Remaining exterior resistance
    Rfloor=3.333,        // Standard floor insulation (U≈0.30 W/(m²·K) for 1 m²)
    RFloorRem=0.130,     // Remaining floor resistance
    Rroof=5.000,         // Good roof insulation (U≈0.20 W/(m²·K) for 1 m²)
    RRoofRem=0.130);     // Remaining roof resistance

  annotation(Documentation(info="<html>
<p><b>Small Residential House Configuration - UPDATED</b></p>

<p><b>Building Characteristics:</b></p>
<ul>
<li>Living area: 50 m²</li>
<li>Cellar area: 40 m²</li>
<li>Roof area: 45 m²</li>
<li>Typical 2-person household</li>
</ul>

<li>
Dec 7, 2025, by Karthik Murugesan
</li>
<p><b>Thermal Properties</b></p>
<p>Standard residential insulation levels:</p>
<ul>
<li><b>Walls:</b> R = 2.857 K/W (U ≈ 0.35 W/(m²·K)) - Standard insulation</li>
<li><b>Floor:</b> R = 3.333 K/W (U ≈ 0.30 W/(m²·K)) - Standard insulation</li>
<li><b>Roof:</b> R = 5.000 K/W (U ≈ 0.20 W/(m²·K)) - Good insulation</li>
<li><b>Remaining resistances:</b> 0.130 K/W - Additional thermal mass effects</li>
</ul>

<li>
Oct 30, 2025, by Karthik Murugesan
</li>
<p><b>Energy Performance:</b></p>
<p>This small house has standard insulation suitable for:</p>
<ul>
<li>Moderate climate zones</li>
<li>Typical residential heating requirements</li>
<li>Good roof insulation to prevent heat loss through the top</li>
</ul>
</html>"));
end SmallHouse;
