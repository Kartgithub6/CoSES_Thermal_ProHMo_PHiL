within CoSES_Thermal_ProHMo_PHiL.Examples;
model TestThreeZoneBuilding_withWeather_ScenarioB
  "Test scenario with STATIC inputs matching VeriStand exactly"

  extends Modelica.Icons.Example;

  // ============================================================================
  // BUILDING MODEL
  // ============================================================================
  CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis.ThreeZoneBuilding_PHiL_optimized_withWeather building(
    redeclare CoSES_Thermal_ProHMo_PHiL.Data.Factory buildingData,
    useWeatherFile   = false,
    T_ambient_fixed  = 275.15,
    TZoneInit_living = 293.85,
    TZoneInit_cellar = 292.15,
    TZoneInit_roof   = 291.15)
    annotation(Placement(visible=true, transformation(origin={3,1},
      extent={{-33,-33},{33,33}}, rotation=0)));

  // ============================================================================
  // OUTPUT VARIABLES
  // ============================================================================


  Real T_living_degC   = building.T_roomIs_degC;

  Real T_cellar_degC   = building.T_cellarIs_degC;

  Real T_roof_degC     = building.T_roofIs_degC;

  Real T_return_degC   = building.STM_HCRL_Set_degC;

  Real qv_lpm          = building.SFW_HCRLbM_Set_l_per_min;

  Real valve_living    = building.valve_living_opening;

  Real valve_cellar    = building.valve_cellar_opening;

  Real valve_roof      = building.valve_roof_opening;

  // Heat flows — positive = heat delivered to zone
  Real Q_heat_living_W = building.building.living_hydSys.radiator.heatPortCon.Q_flow + building.building.living_hydSys.radiator.heatPortRad.Q_flow;

  Real Q_heat_roof_W   = building.building.roof_hydSys.radiator.heatPortCon.Q_flow + building.building.roof_hydSys.radiator.heatPortRad.Q_flow;

  Real Q_heat_cellar_W = building.building.cellar_hydSys.radiator.heatPortCon.Q_flow + building.building.cellar_hydSys.radiator.heatPortRad.Q_flow;


  // Control errors vs VeriStand targets

  Real error_living = 20.7 - T_living_degC;

  Real error_cellar = 19.0 - T_cellar_degC;

  Real error_roof   = 18.0 - T_roof_degC;

  // ============================================================================
  // STATIC INPUTS — matching VeriStand exactly
  // ============================================================================

  Modelica.Blocks.Sources.Constant T_supply_src(k=50)
    annotation(Placement(transformation(origin={-89,75},  extent={{-7,-7},{7,7}})));

  Modelica.Blocks.Sources.Constant flow_src(k=12)
    annotation(Placement(transformation(origin={-89,49},  extent={{-7,-7},{7,7}})));

  Modelica.Blocks.Sources.Constant T_ambient_src(k=2)
    annotation(Placement(transformation(origin={-89,25},  extent={{-7,-7},{7,7}})));

  Modelica.Blocks.Sources.Constant occ_living_src(k=2)
    annotation(Placement(transformation(origin={-89,-3},   extent={{-7,-7},{7,7}})));

  Modelica.Blocks.Sources.Constant occ_cellar_src(k=0)
    annotation(Placement(transformation(origin={-89,-27},  extent={{-7,-7},{7,7}})));

  Modelica.Blocks.Sources.Constant occ_roof_src(k=1)
    annotation(Placement(transformation(origin={-89,-53},  extent={{-7,-7},{7,7}})));

  Modelica.Blocks.Sources.Constant app_living_src(k=300)
    annotation(Placement(transformation(origin={88,2},    extent={{8,-8},{-8,8}})));

  Modelica.Blocks.Sources.Constant app_cellar_src(k=50)
    annotation(Placement(transformation(origin={88,-26},  extent={{8,-8},{-8,8}})));

  Modelica.Blocks.Sources.Constant app_roof_src(k=100)
    annotation(Placement(transformation(origin={88,-54},   extent={{8,-8},{-8,8}})));

equation
  connect(T_supply_src.y, building.STM_HCVLaM_degC) annotation (Line(points={{-81.3,
          75},{-42,75},{-42,30},{-36,30},{-36,30.7},{-33.3,30.7}}, color={0,0,127}));
  connect(flow_src.y, building.SFW_HCRLbM_l_per_min) annotation (Line(points={{-81.3,
          49},{-50,49},{-50,24.1},{-33.3,24.1}}, color={0,0,127}));
  connect(T_ambient_src.y, building.T_ambient_degC) annotation (Line(points={{-81.3,
          25},{-56,25},{-56,17.5},{-33.3,17.5}}, color={0,0,127}));
  connect(occ_living_src.y, building.nPersons_living_in) annotation (Line(
        points={{-81.3,-3},{-54,-3},{-54,7.6},{-33.3,7.6}}, color={0,0,127}));
  connect(occ_cellar_src.y, building.nPersons_cellar_in) annotation (Line(
        points={{-81.3,-27},{-48,-27},{-48,1},{-33.3,1}}, color={0,0,127}));
  connect(occ_roof_src.y, building.nPersons_roof_in) annotation (Line(points={{-81.3,
          -53},{-40,-53},{-40,-5.6},{-33.3,-5.6}}, color={0,0,127}));
  connect(app_living_src.y, building.P_appliances_living_W_in) annotation (Line(
        points={{79.2,2},{54,2},{54,-22.1},{39.3,-22.1}}, color={0,0,127}));
  connect(building.P_appliances_cellar_W_in, app_cellar_src.y) annotation (Line(
        points={{39.3,-25.4},{78,-25.4},{78,-26},{79.2,-26}}, color={0,0,127}));
  connect(building.P_appliances_roof_W_in, app_roof_src.y) annotation (Line(
        points={{39.3,-28.7},{42,-28.7},{42,-28},{56,-28},{56,-54},{79.2,-54}},
        color={0,0,127}));
  annotation(experiment(
    StopTime  = 86400,
    Interval  = 60,
    Tolerance = 1e-06,
    __Dymola_Algorithm = "Dassl"),
    Documentation(info="<html>
<h4>Test Scenario with Weather - Scenario B</h4>
<p><b>✅ Test File - With Weather - Constant Inputs</b></p>
<li>
Feb 28, 2026, by Karthik Murugesan
</li>
<p><b>✅ Simulates for 86400 seconds (24 hours)</b></p>
<p><b>Note: Simulate this model!</b></p>
</html>"));

end TestThreeZoneBuilding_withWeather_ScenarioB;
