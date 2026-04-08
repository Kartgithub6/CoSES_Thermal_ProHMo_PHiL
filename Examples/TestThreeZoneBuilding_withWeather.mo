within CoSES_Thermal_ProHMo_PHiL.Examples;
model TestThreeZoneBuilding_withWeather
  "Test scenario using PHiL wrapper with Weather Data"
  extends Modelica.Icons.Example;

  // ============================================================================
  // 1. BUILDING MODEL - Using PHiL Wrapper WITH WEATHER
  // ============================================================================
  CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis.ThreeZoneBuilding_PHiL_optimized_withWeather building(
    redeclare CoSES_Thermal_ProHMo_PHiL.Data.Factory buildingData,
      weatherDataFile="E:/Documents/Dymola/Library/IBPSA/Resources/weatherdata/DEU_Munich.108660_IWEC.mos",
      useWeatherFile=true,
    T_ambient_fixed=271.15,
    TZoneInit_living=293.15,
    TZoneInit_cellar=291.15,
    TZoneInit_roof=292.15)
      "Building with PHiL interface and weather data"
      annotation(Placement(visible=true, transformation(origin={4,-14},extent={{-60,-60},{60,60}}, rotation=0)));


  // ============================================================================
  // 2. OUTPUT VARIABLES - For Easy Plotting
  // ============================================================================
  // Zone temperatures
  Real T_living_degC = building.T_roomIs_degC "Living room temperature [°C]";
  Real T_cellar_degC = building.T_cellarIs_degC "Cellar temperature [°C]";
  Real T_roof_degC = building.T_roofIs_degC "Roof office temperature [°C]";

  // System variables
  Real T_return_degC = building.STM_HCRL_Set_degC "Return water temperature [°C]";
  Real qv_lpm = building.SFW_HCRLbM_Set_l_per_min "Volume flow rate [L/min]";

  // Valve positions
  Real valve_living = building.valve_living_opening "Living zone valve [0-1]";
  Real valve_cellar = building.valve_cellar_opening "Cellar zone valve [0-1]";
  Real valve_roof = building.valve_roof_opening "Roof zone valve [0-1]";

  // Control errors
  Real error_living = 20.0 - T_living_degC "Living error [K]";
  Real error_cellar = 18.0 - T_cellar_degC "Cellar error [K]";
  Real error_roof = 20.0 - T_roof_degC "Roof error [K]";

  // ============================================================================
  // 3. SUPPLY CONDITIONS - What the boiler/heat source provides
  // ============================================================================
  Modelica.Blocks.Sources.CombiTimeTable T_supply_degC(table=[0,45; 28800,50; 61200,
        50; 64800,45; 86400,45])
      "Supply water temperature from boiler [°C]"
    annotation(Placement(visible=true, transformation(origin={-202,114},extent={{-10,-10},{10,10}}, rotation=0)));
                   // Night: 65°C baseline
                   // 8am: Morning boost (+5°C for faster warmup)
                   // 5pm: Maintain boost
                   // 6pm: Back to baseline
                   // End: Baseline

  // Not used in this simulation (PHiL would provide measured flow)
  Modelica.Blocks.Sources.Constant flow_dummy(k=12)
    "Dummy flow rate (not used, building calculates own flow)"
    annotation(Placement(visible=true, transformation(origin={-202,82}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // 4. OUTDOOR CONDITIONS - Daily temperature cycle
  // ============================================================================
  Modelica.Blocks.Sources.CombiTimeTable T_ambient_degC(
    table=[
      0,-2;
      7200,-2;
      14400,0;
      21600,2;
      28800,3;
      36000,4;
      43200,5;
      50400,4;
      57600,3;
      64800,1;
      72000,-1;
      79200,-2;
      86400,-2])
      "Outdoor ambient temperature cycle [°C]"
    annotation(Placement(visible=true, transformation(origin={-200,42},  extent={{-10,-10},{10,10}}, rotation=0)));
                  // 00:00: Cold night -2°C
                  // 02:00: Coldest point
                  // 04:00: Starting to warm
                  // 06:00: Sunrise warming
                  // 08:00: Morning
                  // 10:00: Mid-morning
                  // 12:00: Peak warmth
                  // 14:00: Afternoon
                  // 16:00: Cooling
                  // 18:00: Evening
                  // 20:00: Night cooling
                  // 22:00: Cold night
                  // 24:00: End of day

  // ============================================================================
  // 5. CCUPANCY SCHEDULES - Realistic daily patterns
  // ============================================================================
  Modelica.Blocks.Sources.CombiTimeTable occupancy_living(table=[0,0; 25200,2;
        28800,1; 43200,1; 64800,2; 75600,3; 82800,2; 86400,0])
      "Living room occupancy [persons]"
    annotation(Placement(visible=true, transformation(origin={-200,-36}, extent={{-10,-10},{10,10}}, rotation=0)));
                  // 00:00-07:00: Everyone sleeping
                  // 07:00: 2 people wake up (breakfast)
                  // 08:00: 1 person leaves (work)
                  // 12:00: 1 person home (lunch)
                  // 18:00: 2 people home (dinner)
                  // 21:00: Visitor arrives (3 people)
                  // 23:00: Visitor leaves (2 people)
                  // 24:00: Everyone sleeping

  Modelica.Blocks.Sources.CombiTimeTable occupancy_cellar(
    table=[
      0,0;
      64800,1;
      68400,0;
      86400,0])
      "Cellar occupancy [persons]"
    annotation(Placement(visible=true, transformation(origin={-200,-72}, extent={{-10,-10},{10,10}}, rotation=0)));
                  // Mostly empty
                  // 18:00: Someone doing laundry
                  // 19:00: Done

  Modelica.Blocks.Sources.CombiTimeTable occupancy_roof(
    table=[
      0,0;
      28800,1;
      43200,1;
      61200,1;
      64800,0;
      86400,0])
      "Roof office occupancy [persons]"
    annotation(Placement(visible=true, transformation(origin={-198,-110}, extent={{-10,-10},{10,10}}, rotation=0)));
                  // Night: Empty
                  // 08:00: Home office starts
                  // 12:00: Lunch break (stays in office)
                  // 17:00: Still working
                  // 18:00: Workday ends

  // ============================================================================
  // 6. APPLIANCE LOADS - Realistic residential usage
  // ============================================================================
  Modelica.Blocks.Sources.CombiTimeTable appliances_living(table=[0,100; 25200,350;
        28800,150; 43200,600; 50400,150; 64800,650; 68400,250; 75600,200; 82800,
        100; 86400,100])
      "Living room appliances [W] - Residential cooking"
    annotation(Placement(visible=true, transformation(origin={202,-24}, extent={{10,-10},{-10,10}}, rotation=0)));
                    // 00:00: Fridge baseline
                    // 07:00: Breakfast (kettle + toaster)
                    // 08:00: Back to baseline
                    // 12:00: Lunch cooking
                    // 14:00: Baseline
                    // 18:00: Dinner cooking
                    // 19:00: TV + lights
                    // 21:00: Evening electronics
                    // 23:00: Night (fridge only)
                    // 24:00: Night

  Modelica.Blocks.Sources.CombiTimeTable appliances_cellar(table=[0,50; 36000,100;
        43200,50; 86400,50])
      "Cellar appliances [W]"
    annotation(Placement(visible=true, transformation(origin={202,-60}, extent={{10,-10},{-10,10}}, rotation=0)));
                    // Baseline (freezer)
                    // Baseline (freezer)
                    // 12:00: Done

  Modelica.Blocks.Sources.CombiTimeTable appliances_roof(table=[0,50; 28800,200;
        61200,50; 86400,50])
      "Roof office appliances [W]"
    annotation(Placement(visible=true, transformation(origin={202,-102},extent={{10,-10},{-10,10}}, rotation=0)));
                    // Night
                    // 08:00: Computer + monitors + printer
                    // 17:00: Workday ends

  // Heat flow outputs — positive convention (heat delivered TO zone)
  Real Q_heat_living_W = -(building.building.living_hydSys.radiator.heatPortCon.Q_flow
                       + building.building.living_hydSys.radiator.heatPortRad.Q_flow)
    "Heat delivered to living zone [W] — positive = heating";
  Real Q_heat_roof_W   = -(building.building.roof_hydSys.radiator.heatPortCon.Q_flow
                       + building.building.roof_hydSys.radiator.heatPortRad.Q_flow)
    "Heat delivered to roof zone [W] — positive = heating";
  Real Q_heat_cellar_W = -(building.building.cellar_hydSys.radiator.heatPortCon.Q_flow
                       + building.building.cellar_hydSys.radiator.heatPortRad.Q_flow)
    "Heat delivered to cellar zone [W] — positive = heating";

equation
  // ═══════════════════════════════════════════════════════════════════
  // 7. PHiL INPUT CONNECTIONS - From Environment to Building
  // ═══════════════════════════════════════════════════════════════════
  connect(T_supply_degC.y[1], building.STM_HCVLaM_degC)
    annotation(Line(points={{-191,114},{-62,114},{-62,40}},          color={0,0,127}));

  connect(flow_dummy.y, building.SFW_HCRLbM_l_per_min)
    annotation(Line(points={{-191,82},{-80,82},{-80,28},{-62,28}},   color={0,0,127}));

  connect(T_ambient_degC.y[1], building.T_ambient_degC)
    annotation(Line(points={{-189,42},{-84,42},{-84,16},{-62,16}},     color={0,0,127}));

  // ============================================================================
  // 8. OCCUPANCY INPUT CONNECTIONS
  // ============================================================================
  connect(occupancy_living.y[1], building.nPersons_living_in)
    annotation(Line(points={{-189,-36},{-142,-36},{-142,-2},{-62,-2}}, color={0,0,127}));

  connect(occupancy_cellar.y[1], building.nPersons_cellar_in)
    annotation(Line(points={{-189,-72},{-120,-72},{-120,-14},{-62,-14}},
                                                                       color={0,0,127}));

  connect(occupancy_roof.y[1], building.nPersons_roof_in)
    annotation(Line(points={{-187,-110},{-100,-110},{-100,-26},{-62,-26}}, color={0,0,127}));

  // ============================================================================
  // 9. APPLIANCE LOAD CONNECTIONS
  // ============================================================================
  connect(appliances_living.y[1], building.P_appliances_living_W_in)
    annotation(Line(points={{191,-24},{80,-24},{80,-56},{70,-56}}, color={0,0,127}));

  connect(appliances_cellar.y[1], building.P_appliances_cellar_W_in)
    annotation(Line(points={{191,-60},{128,-60},{128,-62},{70,-62}},
                                                                   color={0,0,127}));

  connect(appliances_roof.y[1], building.P_appliances_roof_W_in)
    annotation(Line(points={{191,-102},{82,-102},{82,-68},{70,-68}}, color={0,0,127}));

  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-200},{200,200}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-200},{200,200}})),
    experiment(
      StopTime=86400,
      Interval=60,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<h4>Test Scenario with Weather</h4>
<p><b>✅ Test File - With Weather</b></p>
<li>
Feb 28, 2026, by Karthik Murugesan
</li>
<p>Changed the following inputs </p>
<ul>
<li>T_living_degC, T_cellar_degC, T_roof_degC</li>
<li>T_return_degC</li>
<li>qv_lpm - Volume flow rate (Calculated by building)</li>
<li>valve_living, valve_cellar, valve_roof</li>
<li>error_living, error_cellar, error_roof</li>
</ul>
<li>
Jan 20, 2026, by Karthik Murugesan
</li>
<p><b>✅ Weather data Enabled</b></p>
<p><b>✅ CombiTimeTables for all inputs</b></p>
<p><b>✅ Simulates for 86400 seconds (24 hours)</b></p>

<p><b>Note: Simulate this model!</b></p>
</html>"));
end TestThreeZoneBuilding_withWeather;
