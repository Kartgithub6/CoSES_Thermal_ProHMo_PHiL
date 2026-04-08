within CoSES_Thermal_ProHMo_PHiL.Data;
record Hotel "Hotel building"
  extends BaseBuilding(
    // Zone Areas - Hotel layout
    AZone_living=180.0,  // Guest rooms
    AZone_cellar=120.0,  // Facilities/storage
    AZone_roof=160.0,    // Additional guest rooms

    // Zone Heights - Hotel standard
    hZone_living=3.0,
    hZone_cellar=2.8,
    hZone_roof=3.0,

    // Initial Temperatures - Hotel comfort (high)
    TZoneInit_living=295.15,  // 22°C
    TZoneInit_cellar=291.15,  // 18°C
    TZoneInit_roof=294.15,    // 21°C

    // Reference Temperatures - Guest comfort priority
    TRef_living=296.15,  // 23°C - Guest rooms
    TRef_cellar=291.15,  // 18°C - Facilities
    TRef_roof=295.15,    // 22°C - Guest rooms

    // Control Parameters - Tight control for guest comfort
    k_PI=0.38,
    Ti_PI=350,
    yMin_living=0.06,
    yMin_cellar=0.12,
    yMin_roof=0.07,

    // Heating
    cellarHeat=false,
    roofHeat=true,

    // Internal Loads - Hotel amenities
    P_appliances_living_default=2000,  // HVAC, lighting, appliances
    P_appliances_cellar_default=800,   // Laundry, kitchen
    P_appliances_roof_default=1800,    // Guest room systems

    // Occupancy - Guests + staff
    nPersons_living_default=25,
    nPersons_cellar_default=10,
    nPersons_roof_default=20,

    // Added: Thermal Resistances for Hotel
    // Hotels need good insulation for guest comfort and energy efficiency
    Rwall=3.571,         // Good wall insulation (U≈0.28 W/(m²·K) for 1 m²)
    RExtRem=0.145,       // Remaining exterior resistance
    Rfloor=4.000,        // Good floor insulation (U≈0.25 W/(m²·K) for 1 m²)
    RFloorRem=0.145,     // Remaining floor resistance
    Rroof=6.250,         // Excellent roof insulation (U≈0.16 W/(m²·K) for 1 m²)
    RRoofRem=0.145);     // Remaining roof resistance

  annotation(Documentation(info="<html>
<p><b>Hotel Building Configuration - NEW</b></p>

<p><b>Building Characteristics:</b></p>
<ul>
<li>Guest rooms: 180 m²</li>
<li>Facilities/storage: 120 m²</li>
<li>Additional rooms: 160 m²</li>
<li>High comfort standards for guests</li>
<li>24/7 operation</li>
<li>Variable occupancy</li>
</ul>

<li>
Dec 7, 2025, by Karthik Murugesan
</li>
<p><b>Thermal Properties</b></p>
<p>Hospitality-grade insulation:</p>
<ul>
<li><b>Walls:</b> R = 3.571 K/W (U ≈ 0.28 W/(m²·K)) - Good insulation</li>
<li><b>Floor:</b> R = 4.000 K/W (U ≈ 0.25 W/(m²·K)) - Good insulation</li>
<li><b>Roof:</b> R = 6.250 K/W (U ≈ 0.16 W/(m²·K)) - Excellent insulation</li>
</ul>

<li>
Oct 30, 2025, by Karthik Murugesan
</li>
<p><b>Energy Performance:</b></p>
<ul>
<li>Guest comfort priority</li>
<li>Energy efficiency for 24/7 operation</li>
<li>Good sound insulation</li>
</ul>
</html>"));
end Hotel;
