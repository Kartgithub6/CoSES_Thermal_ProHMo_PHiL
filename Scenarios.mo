within CoSES_Thermal_ProHMo_PHiL;
package Scenarios
  model SF1_Dymola
    "SF1: Solar Thermal + CHP + CB + Building + DHW - Complete System"

    // ============================================================================
    // SYSTEM PARAMETERS
    // ============================================================================
    parameter Modelica.Units.SI.Area ST_AModule = 1.312 "Module area [m²]";
    parameter Modelica.Units.SI.Power CHP_QHeat_nom = 5300 "CHP thermal [W]";
    parameter Modelica.Units.SI.Power CHP_PElec_nom = 2000 "CHP electrical [W]";
    parameter Modelica.Units.SI.Power CB_QHeat_nom = 14000 "CB thermal [W]";

    parameter Modelica.Units.SI.Volume V_buffer = 0.8 "Buffer volume [m³]";
    parameter Modelica.Units.SI.Temperature T_buffer_init = 338.15 "Initial 65°C";
    parameter Modelica.Units.SI.Temperature T_buffer_max = 353.15 "Max 80°C";
    parameter Modelica.Units.SI.Temperature T_buffer_min = 313.15 "Min 40°C";

    parameter Modelica.Units.SI.Volume V_DHW = 0.5 "DHW volume [m³]";
    parameter Modelica.Units.SI.Temperature T_DHW_init = 328.15 "Initial 55°C";
    parameter Modelica.Units.SI.Temperature T_DHW_set = 318.15 "Setpoint 45°C";

    // ============================================================================
    // CONTROL PARAMETERS
    // ============================================================================
    parameter Real SOC_CHP_on = 0.55 "CHP ON when SOC < 55%";
    parameter Real SOC_CHP_off = 0.90 "CHP OFF when SOC > 90%";
    parameter Real SOC_CB_on = 0.40 "CB ON when SOC < 40%";
    parameter Real SOC_CB_off = 0.60 "CB OFF when SOC > 60%";

    parameter Real deltaTon_ST = 5 "ST ON when deltaT > 5K";
    parameter Real deltaToff_ST = 2 "ST OFF when deltaT < 2K";
    parameter Modelica.Units.SI.Irradiance GHI_min = 100 "Min GHI for ST [W/m²]";

    parameter Modelica.Units.SI.Temperature T_DHW_pump_on = 318.15 "DHW pump ON < 45°C";
    parameter Modelica.Units.SI.Temperature T_DHW_pump_off = 333.15 "DHW pump OFF > 60°C";
    parameter Modelica.Units.SI.Temperature T_buffer_DHW_min = 333.15 "Min buffer for DHW 60°C";

    // ============================================================================
    // THERMAL PARAMETERS
    // ============================================================================
    parameter Real cp = 4180 "Water specific heat [J/(kg·K)]";
    parameter Real rho = 1000 "Water density [kg/m³]";
    parameter Real C_buffer = rho * cp * V_buffer "Buffer capacity [J/K]";
    parameter Real C_DHW = rho * cp * V_DHW "DHW capacity [J/K]";
    parameter Real UA_buffer = 5 "Buffer loss [W/K]";
    parameter Real UA_DHW = 3 "DHW loss [W/K]";
    parameter Real qv_heating = 6 "Heating flow [L/min]";

    // ============================================================================
    // COMPONENT INSTANTIATION
    // ============================================================================

    ThermoEnergeticAnalysis.ThreeZoneBuilding_PHiL buildingModel(
      AZone_cellar = 80,
      AZone_living = 120,
      AZone_roof = 100,
      TZoneInit_living = 294.15,
      TZoneInit_cellar = 288.15,
      TZoneInit_roof = 291.15,
      cellarHeat = true,
      roofHeat = true)
      annotation(Placement(transformation(extent={{160,-20},{220,40}})));

    HeatGenerators.SimpleCHP_v2 chp(
      QHeat_nominal = CHP_QHeat_nom,
      PElec_nominal = CHP_PElec_nom,
      eta_heat = 0.65,
      eta_elec = 0.25,
      ModulationMin = 0.5,
      tau = 60)
      annotation(Placement(transformation(extent={{-20,60},{40,120}})));

    HeatGenerators.SimpleCondensingBoiler cb(
      QHeat_nominal = CB_QHeat_nom,
      eta_nominal = 0.98,
      eta_noncondensing = 0.88,
      ModulationMin = 0.2,
      tau = 30)
      annotation(Placement(transformation(extent={{-20,-20},{40,40}})));

    HeatGenerators.SimpleSolarThermal st(
      nSeries = 3,
      nParallel = 3,
      AModule = ST_AModule,
      etaOptical = 0.642,
      a1 = 0.885,
      a2 = 0.001,
      TCollectorInit = 293.15)
      annotation(Placement(transformation(extent={{-20,-100},{40,-40}})));

    Consumer.SimpleDHW dhw(
      THotWaterSet = T_DHW_set,
      TColdWaterConst = 283.15,
      DHWfactor = 1.0)
      annotation(Placement(transformation(extent={{160,-100},{220,-40}})));

    // ============================================================================
    // HYSTERESIS CONTROLLERS
    // ============================================================================
    Modelica.Blocks.Logical.Hysteresis hysCHP(
      uLow = SOC_CHP_on,
      uHigh = SOC_CHP_off,
      pre_y_start = true)
      "CHP hysteresis on buffer SOC"
      annotation(Placement(transformation(extent={{60,80},{80,100}})));

    Modelica.Blocks.Logical.Hysteresis hysCB(
      uLow = SOC_CB_on,
      uHigh = SOC_CB_off,
      pre_y_start = false)
      "CB hysteresis on buffer SOC"
      annotation(Placement(transformation(extent={{60,0},{80,20}})));

    Modelica.Blocks.Logical.Hysteresis hysST(
      uLow = deltaToff_ST,
      uHigh = deltaTon_ST,
      pre_y_start = false)
      "ST hysteresis on temperature difference"
      annotation(Placement(transformation(extent={{60,-80},{80,-60}})));

    Modelica.Blocks.Logical.Hysteresis hysDHW(
      uLow = T_DHW_pump_on,
      uHigh = T_DHW_pump_off,
      pre_y_start = false)
      "DHW pump hysteresis"
      annotation(Placement(transformation(extent={{100,-60},{120,-40}})));

    // ============================================================================
    // WEATHER DATA
    // ============================================================================
    Modelica.Blocks.Sources.CombiTimeTable weatherData(
      tableOnFile = false,
      table = [
        0,       -3.0,   0;
        7200,    -5.0,   0;
        14400,   -5.2,   0;
        18000,   -4.0,   10;
        21600,   -2.5,   50;
        25200,   -1.0,   150;
        28800,   0.5,    300;
        32400,   1.8,    450;
        36000,   2.5,    550;
        39600,   2.8,    600;
        43200,   3.0,    620;
        46800,   2.5,    580;
        50400,   2.0,    480;
        54000,   1.5,    350;
        57600,   0.5,    180;
        61200,   -0.5,   60;
        64800,   -1.5,   10;
        72000,   -3.2,   0;
        79200,   -4.2,   0;
        86400,   -4.8,   0;
        172800,  -3.5,   0;
        259200,  -2.0,   0;
        345600,  -1.0,   0;
        432000,  -8.0,   0;
        518400,  -9.0,   0;
        604800,  -5.0,   0;
        691200,  -2.0,   0;
        777600,  -1.0,   0;
        864000,  0.0,    0],
      columns = {2,3},
      smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments)
      annotation(Placement(transformation(extent={{-180,100},{-160,120}})));

    Modelica.Blocks.Sources.CombiTimeTable occupancyData(
      tableOnFile = false,
      table = [
        0,      6, 0, 0, 100,  50, 30;
        21600,  6, 0, 0, 400,  50, 30;
        28800,  2, 0, 0, 150,  50, 50;
        43200,  2, 0, 0, 200,  50, 50;
        64800,  6, 0, 0, 600,  50, 30;
        75600,  6, 0, 0, 300,  50, 30;
        82800,  6, 0, 0, 100,  50, 30;
        86400,  6, 0, 0, 100,  50, 30],
      columns = {2,3,4,5,6,7},
      extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic,
      smoothness = Modelica.Blocks.Types.Smoothness.ConstantSegments)
      annotation(Placement(transformation(extent={{-180,60},{-160,80}})));

    Modelica.Blocks.Sources.CombiTimeTable dhwDemand(
      tableOnFile = false,
      table = [
        0,      0;
        21600,  60;
        25200,  30;
        28800,  10;
        43200,  20;
        64800,  50;
        72000,  40;
        79200,  20;
        82800,  5;
        86400,  0],
      columns = {2},
      extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic,
      smoothness = Modelica.Blocks.Types.Smoothness.ConstantSegments)
      annotation(Placement(transformation(extent={{-180,20},{-160,40}})));

    // ============================================================================
    // STATE VARIABLES
    // ============================================================================
    Modelica.Units.SI.Temperature T_buffer(start=T_buffer_init, fixed=true);
    Modelica.Units.SI.Temperature T_DHW_storage(start=T_DHW_init, fixed=true);
    Real SOC_buffer;

    // ============================================================================
    // CONTROL SIGNALS
    // ============================================================================
    Boolean CHP_on, CB_on, ST_on, DHW_pump_on;
    Real Mod_CHP, Mod_CB;
    Real deltaT_ST "Temperature difference for ST control";

    // ============================================================================
    // INTERNAL VARIABLES
    // ============================================================================
    Real T_ambient_degC, T_ambient_K, GHI;
    Real Q_ST, Q_CHP, Q_CB;
    Real Q_to_buffer, Q_to_building, Q_to_DHW;
    Real Q_loss_buffer, Q_loss_DHW, Q_DHW_demand;
    Real T_supply_degC, T_return_degC, T_buffer_degC, T_DHW_degC;
    Real T_living_degC, T_cellar_degC, T_roof_degC;
    Real deltaT_building, deltaT_DHW;

    // ============================================================================
    // MONITORING OUTPUTS
    // ============================================================================
    Real Q_ST_kW, Q_CHP_kW, Q_CB_kW, P_CHP_elec_kW, Q_heating_kW, Q_DHW_kW;
    Real E_ST_kWh(start=0, fixed=true);
    Real E_CHP_th_kWh(start=0, fixed=true);
    Real E_CHP_el_kWh(start=0, fixed=true);
    Real E_CB_kWh(start=0, fixed=true);
    Real E_heating_kWh(start=0, fixed=true);
    Real E_DHW_kWh(start=0, fixed=true);

    Real comfort_deviation(start=0, fixed=true);
    Real comfort_pct;
    Real ST_runtime_h(start=0, fixed=true);
    Real CHP_runtime_h(start=0, fixed=true);
    Real CB_runtime_h(start=0, fixed=true);

    Real time_hours, time_days;

  equation
    // ============================================================================
    // WEATHER DATA
    // ============================================================================
    T_ambient_degC = weatherData.y[1];
    T_ambient_K = T_ambient_degC + 273.15;
    GHI = weatherData.y[2];

    // ============================================================================
    // BUILDING INPUTS/OUTPUTS
    // ============================================================================
    buildingModel.nPersons_living_in = occupancyData.y[1];
    buildingModel.nPersons_cellar_in = occupancyData.y[2];
    buildingModel.nPersons_roof_in = occupancyData.y[3];
    buildingModel.P_appliances_living_W_in = occupancyData.y[4];
    buildingModel.P_appliances_cellar_W_in = occupancyData.y[5];
    buildingModel.P_appliances_roof_W_in = occupancyData.y[6];
    buildingModel.T_ambient_degC = T_ambient_degC;

    T_living_degC = buildingModel.T_roomIs_degC;
    T_cellar_degC = buildingModel.T_cellarIs_degC;
    T_roof_degC = buildingModel.T_roofIs_degC;
    T_return_degC = buildingModel.STM_HCRL_Set_degC;
    T_buffer_degC = T_buffer - 273.15;
    T_DHW_degC = T_DHW_storage - 273.15;

    // ============================================================================
    // BUFFER SOC
    // ============================================================================
    SOC_buffer = (T_buffer - T_buffer_min) / (T_buffer_max - T_buffer_min);

    // ============================================================================
    // HYSTERESIS CONTROLLER CONNECTIONS
    // ============================================================================
    // CHP control
    hysCHP.u = SOC_buffer;
    CHP_on = not hysCHP.y;

    // CB control
    hysCB.u = SOC_buffer;
    CB_on = not hysCB.y;

    // ST control - based on temperature difference and GHI
    deltaT_ST = st.TCollector_degC - T_buffer_degC;
    hysST.u = deltaT_ST;
    ST_on = hysST.y and (GHI > GHI_min);

    // DHW pump control
    hysDHW.u = T_DHW_storage;
    DHW_pump_on = (not hysDHW.y) and (T_buffer > T_buffer_DHW_min);

    // ============================================================================
    // MODULATION
    // ============================================================================
    Mod_CHP = if CHP_on then max(0.5, min(1.0, 1.5 - SOC_buffer)) else 0;
    Mod_CB = if CB_on then max(0.2, min(1.0, 1.3 - SOC_buffer)) else 0;

    // ============================================================================
    // CHP CONNECTIONS
    // ============================================================================
    chp.CHPon = CHP_on;
    chp.Modulation = Mod_CHP;
    chp.TReturn_degC = T_buffer_degC - 15;
    chp.qv_l_per_min = if CHP_on then 8.0 else 0.001;
    Q_CHP = chp.QHeat_kW * 1000;

    // ============================================================================
    // CB CONNECTIONS
    // ============================================================================
    cb.CBon = CB_on;
    cb.Modulation = Mod_CB;
    cb.TReturn_degC = T_buffer_degC - 15;
    cb.qv_l_per_min = if CB_on then 10.0 else 0.001;
    Q_CB = cb.QHeat_kW * 1000;

    // ============================================================================
    // SOLAR THERMAL CONNECTIONS
    // ============================================================================
    st.STon = ST_on;
    st.qvRef = if ST_on then 0.0002 else 0.00001;
    st.TReturn_degC = T_buffer_degC - 10;
    st.TAmbient_degC = T_ambient_degC;
    st.GHI = GHI;
    Q_ST = st.QSolarThermal;

    // ============================================================================
    // BUFFER STORAGE
    // ============================================================================
    Q_to_buffer = Q_ST + Q_CHP + Q_CB;
    Q_loss_buffer = UA_buffer * (T_buffer - T_ambient_K - 15);

    deltaT_building = T_buffer - (T_return_degC + 273.15);
    Q_to_building = if deltaT_building > 5 then
                      deltaT_building * qv_heating / 60 * rho * cp
                    else 0;

    deltaT_DHW = T_buffer - T_DHW_storage;
    Q_to_DHW = if DHW_pump_on and (deltaT_DHW > 5) then
                 min(5000, deltaT_DHW * 300)
               else 0;

    C_buffer * der(T_buffer) = Q_to_buffer - Q_to_building - Q_to_DHW - Q_loss_buffer;
    T_supply_degC = T_buffer_degC;

    // ============================================================================
    // BUILDING SUPPLY
    // ============================================================================
    buildingModel.STM_HCVLaM_degC = T_supply_degC;
    buildingModel.SFW_HCRLbM_l_per_min = qv_heating;

    // ============================================================================
    // DHW STORAGE
    // ============================================================================
    Q_loss_DHW = UA_DHW * (T_DHW_storage - T_ambient_K - 15);
    Q_DHW_demand = dhw.Q_DHW_W;
    C_DHW * der(T_DHW_storage) = Q_to_DHW - Q_DHW_demand - Q_loss_DHW;

    dhw.qv_DHW_demand_Lph = dhwDemand.y[1];
    dhw.TStorage_degC = T_DHW_degC;
    dhw.TColdWater_degC = 10;

    // ============================================================================
    // MONITORING
    // ============================================================================
    Q_ST_kW = Q_ST / 1000;
    Q_CHP_kW = chp.QHeat_kW;
    Q_CB_kW = cb.QHeat_kW;
    P_CHP_elec_kW = chp.PElec_kW;
    Q_heating_kW = Q_to_building / 1000;
    Q_DHW_kW = Q_DHW_demand / 1000;

    der(E_ST_kWh) = Q_ST_kW / 3600;
    der(E_CHP_th_kWh) = Q_CHP_kW / 3600;
    der(E_CHP_el_kWh) = P_CHP_elec_kW / 3600;
    der(E_CB_kWh) = Q_CB_kW / 3600;
    der(E_heating_kWh) = Q_heating_kW / 3600;
    der(E_DHW_kWh) = Q_DHW_kW / 3600;

    der(comfort_deviation) = if T_living_degC < 18 then (18 - T_living_degC)/3600
                             elseif T_living_degC > 24 then (T_living_degC - 24)/3600
                             else 0;
    comfort_pct = if time > 0 then max(0, 100 - comfort_deviation * 10) else 100;

    der(ST_runtime_h) = (if ST_on then 1.0 else 0.0) / 3600;
    der(CHP_runtime_h) = (if CHP_on then 1.0 else 0.0) / 3600;
    der(CB_runtime_h) = (if CB_on then 1.0 else 0.0) / 3600;

    time_hours = time / 3600;
    time_days = time / 86400;

    annotation(
      experiment(StartTime=0, StopTime=86400, Tolerance=1e-05, Interval=60),
      Documentation(info="<html>
<h4>SF1_Dymola - Complete System Model</h4>
<p>Solar Thermal + CHP + CB + Building + DHW</p>
</html>"),
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-120},{240,140}}),
        graphics={
          Line(points={{40,90},{60,90}}, color={255,0,0}, thickness=0.5),
          Line(points={{80,90},{100,90},{100,50},{160,50},{160,30}}, color={255,0,0}, thickness=0.5),
          Text(extent={{85,55},{115,45}}, textString="CHP", textColor={255,0,0}),
          Line(points={{40,10},{60,10}}, color={0,0,255}, thickness=0.5),
          Line(points={{80,10},{100,10},{100,20},{160,20}}, color={0,0,255}, thickness=0.5),
          Text(extent={{85,25},{115,15}}, textString="CB", textColor={0,0,255}),
          Line(points={{40,-70},{60,-70}}, color={255,200,0}, thickness=0.5),
          Line(points={{80,-70},{100,-70},{100,-30},{130,-30}}, color={255,200,0}, thickness=0.5),
          Text(extent={{85,-25},{115,-35}}, textString="ST", textColor={255,200,0}),
          Line(points={{130,0},{160,0}}, color={238,46,47}, thickness=1, pattern=LinePattern.Dash),
          Text(extent={{132,5},{158,-5}}, textString="Heat", textColor={238,46,47}),
          Line(points={{120,-50},{160,-50},{160,-70}}, color={0,140,72}, thickness=0.5),
          Text(extent={{125,-45},{155,-55}}, textString="DHW", textColor={0,140,72}),
          Line(points={{-160,110},{-100,110},{-100,130},{20,130},{20,120}}, color={128,128,128}, thickness=0.25),
          Line(points={{-100,130},{180,130},{180,40}}, color={128,128,128}, thickness=0.25, pattern=LinePattern.Dot),
          Line(points={{-160,70},{-80,70},{-80,140},{200,140},{200,40}}, color={128,128,128}, thickness=0.25, pattern=LinePattern.Dot),
          Ellipse(extent={{120,-20},{140,0}}, lineColor={238,46,47}, fillColor={255,200,200}, fillPattern=FillPattern.Solid),
          Text(extent={{122,-5},{138,-15}}, textString="Buffer", textColor={238,46,47}, textStyle={TextStyle.Bold})}
          // Background rectangles for component groups

          // CONNECTION LINES - CHP to Hysteresis to Building

          // CONNECTION LINES - CB to Hysteresis to Building

          // CONNECTION LINES - ST to Hysteresis to Buffer

          // CONNECTION LINES - Buffer to Building

          // CONNECTION LINES - DHW

          // CONNECTION LINES - Weather to components

          // CONNECTION LINES - Occupancy to Building

          // Buffer Storage symbol
),    Icon(graphics={
        Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0},
          fillColor={255,255,255}, fillPattern=FillPattern.Solid),
        Rectangle(extent={{-80,80},{-20,40}}, lineColor={255,128,0}, fillColor={255,200,100}, fillPattern=FillPattern.Solid),
        Text(extent={{-75,70},{-25,50}}, textString="CHP"),
        Rectangle(extent={{-80,30},{-20,-10}}, lineColor={0,0,255}, fillColor={200,200,255}, fillPattern=FillPattern.Solid),
        Text(extent={{-75,20},{-25,0}}, textString="CB"),
        Rectangle(extent={{-80,-20},{-20,-60}}, lineColor={255,200,0}, fillColor={255,255,150}, fillPattern=FillPattern.Solid),
        Text(extent={{-75,-30},{-25,-50}}, textString="ST"),
        Polygon(points={{20,60},{80,60},{80,-20},{60,-20},{60,0},{40,0},{40,-20},{20,-20},{20,60}},
          lineColor={0,128,0}, fillColor={200,255,200}, fillPattern=FillPattern.Solid),
        Text(extent={{30,40},{70,20}}, textString="Building"),
        Rectangle(extent={{20,-40},{80,-80}}, lineColor={0,140,72}, fillColor={200,255,220}, fillPattern=FillPattern.Solid),
        Text(extent={{25,-50},{75,-70}}, textString="DHW"),
        Line(points={{-20,60},{20,40}}, color={255,0,0}, thickness=1),
        Line(points={{-20,10},{20,20}}, color={0,0,255}, thickness=1),
        Line(points={{-20,-40},{0,-40},{0,-60},{20,-60}}, color={0,140,72}, thickness=1),
        Text(extent={{-90,95},{90,85}}, textColor={0,0,255}, textString="SF1"),
        Text(extent={{-90,-85},{90,-95}}, textString="ST+CHP+CB+Building+DHW")}));
  end SF1_Dymola;

  model MF5_Dymola
    "MF5: CHP + CB + Building + DHW - Complete Component-Based Model (No Solar)"

    // ============================================================================
    // MODEL DESCRIPTION
    // ============================================================================
    // Complete MF5 system model that INSTANTIATES and CONNECTS:
    //   - ThreeZoneBuilding_PHiL (building with hydronic system)
    //   - SimpleCHP_v2 (CHP unit - 12.5kW th / 5kW el) - PRIMARY
    //   - SimpleCondensingBoiler (backup boiler - 50kW)
    //   - SimpleDHW (domestic hot water demand)
    //   - DHW Storage (simplified model)
    //
    // System Architecture:
    //   CHP ────────────────┐
    //                       ├──► Heating Circuit ──┬──► Building (240 m²)
    //   CB (backup) ────────┘                      │
    //                                              └──► DHW Storage ──► DHW Demand
    //
    // Note: MF5 has NO solar thermal (unlike SF1)
    // ============================================================================

    // ============================================================================
    // MODE SELECTION
    // ============================================================================
    parameter Boolean PHiLMode = false
      "true = PHiL mode (external inputs), false = Standalone simulation";

    // ============================================================================
    // SYSTEM PARAMETERS
    // ============================================================================

    // Building (Multi-Family: 240 m²)
    parameter Modelica.Units.SI.Area ABuilding = 240 "Total heated area [m²]";

    // CHP (larger unit for multi-family)
    parameter Modelica.Units.SI.Power CHP_QHeat_nom = 12500 "CHP thermal [W]";
    parameter Modelica.Units.SI.Power CHP_PElec_nom = 5000 "CHP electrical [W]";

    // Condensing Boiler (50 kW backup)
    parameter Modelica.Units.SI.Power CB_QHeat_nom = 50000 "CB thermal [W]";

    // DHW Storage (300L for multi-family)
    parameter Modelica.Units.SI.Volume V_DHW = 0.3 "DHW volume [m³]";
    parameter Modelica.Units.SI.Temperature T_DHW_init = 328.15 "Initial 55°C";
    parameter Modelica.Units.SI.Temperature T_DHW_set = 318.15 "Setpoint 45°C";

    // ============================================================================
    // CONTROL PARAMETERS (Temperature-based with hysteresis)
    // ============================================================================
    // CHP Control
    parameter Modelica.Units.SI.Temperature T_CHP_on = 291.15 "CHP ON when T < 18°C";
    parameter Modelica.Units.SI.Temperature T_CHP_off = 296.15 "CHP OFF when T > 23°C";

    // CB Control (emergency backup)
    parameter Modelica.Units.SI.Temperature T_CB_on = 287.15 "CB ON when T < 14°C";
    parameter Modelica.Units.SI.Temperature T_CB_off = 291.15 "CB OFF when T > 18°C";

    // Temperature setpoint
    parameter Modelica.Units.SI.Temperature T_setpoint = 294.15 "Target 21°C";

    // DHW Pump Control
    parameter Modelica.Units.SI.Temperature T_DHW_pump_on = 318.15 "DHW pump ON < 45°C";
    parameter Modelica.Units.SI.Temperature T_DHW_pump_off = 333.15 "DHW pump OFF > 60°C";

    // ============================================================================
    // THERMAL PARAMETERS
    // ============================================================================
    parameter Real cp = 4180 "Water specific heat [J/(kg·K)]";
    parameter Real rho = 1000 "Water density [kg/m³]";
    parameter Real C_DHW = rho * cp * V_DHW "DHW capacity [J/K]";
    parameter Real UA_DHW = 3 "DHW loss [W/K]";

    // ============================================================================
    // COMPONENT INSTANTIATION
    // ============================================================================

    // THE BUILDING
    CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis.ThreeZoneBuilding_PHiL_optimized building(
      AZone_cellar = 80,
      AZone_living = 100,
      AZone_roof = 60,
      TZoneInit_living = 294.15,
      TZoneInit_cellar = 288.15,
      TZoneInit_roof = 291.15,
      cellarHeat = true,
      roofHeat = true)
      "Three-zone building with hydronic heating (240 m²)"
      annotation(Placement(transformation(extent={{36,-28},{116,52}})));

    // CHP UNIT (PRIMARY)
    CoSES_Thermal_ProHMo_PHiL.HeatGenerators.SimpleCHP_v2 chp(
      QHeat_nominal = CHP_QHeat_nom,
      PElec_nominal = CHP_PElec_nom,
      eta_heat = 0.60,
      eta_elec = 0.25,
      ModulationMin = 0.3,
      tau = 60)
      "CHP - Primary heat source"
      annotation(Placement(transformation(extent={{-100,40},{-60,80}})));

    // CONDENSING BOILER (BACKUP)
    CoSES_Thermal_ProHMo_PHiL.HeatGenerators.SimpleCondensingBoiler cb(
      QHeat_nominal = CB_QHeat_nom,
      eta_nominal = 0.98,
      eta_noncondensing = 0.88,
      ModulationMin = 0.2,
      tau = 30)
      "CB - Backup heat source (50kW)"
      annotation(Placement(transformation(extent={{-100,-20},{-60,20}})));

    // DHW DEMAND
    CoSES_Thermal_ProHMo_PHiL.Consumer.SimpleDHW dhw(
      THotWaterSet = T_DHW_set,
      TColdWaterConst = 283.15,
      DHWfactor = 1.5   // Higher for multi-family
)     "DHW demand"
      annotation(Placement(transformation(extent={{80,-100},{120,-60}})));

    // ============================================================================
    // WEATHER DATA (Munich TRY Winter - 10 days)
    // ============================================================================
    Modelica.Blocks.Sources.CombiTimeTable weatherData(
      tableOnFile = false,
      table = [
        0,       -3.0;
        7200,    -5.0;
        14400,   -5.2;
        21600,   -2.5;
        28800,   0.5;
        36000,   2.5;
        43200,   2.5;
        50400,   1.5;
        57600,   -0.5;
        64800,   -2.5;
        72000,   -3.2;
        79200,   -4.2;
        86400,   -5.2;
        172800,  -4.5;
        259200,  -2.0;
        345600,  -1.0;
        432000,  -9.0;
        518400,  -9.5;
        604800,  -4.5;
        691200,  -2.0;
        777600,  -1.0;
        864000,  0.0],
        // time[s], T_ambient[°C]
                             // Day 2
                             // Day 3
                             // Day 4
                             // Day 5
                             // Day 6: Extreme cold
                             // Day 7
                             // Day 8
                             // Day 9
                             // Day 10
      columns = {2},
      smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments)
      "Weather: T_ambient[°C]"
      annotation(Placement(transformation(extent={{-180,100},{-160,120}})));

    // ============================================================================
    // OCCUPANCY DATA
    // ============================================================================
    Modelica.Blocks.Sources.CombiTimeTable occupancyData(
      tableOnFile = false,
      table = [
        0,      2, 0, 0, 30,  80, 20;
        21600,  2, 0, 0, 200, 80, 20;
        28800,  0, 0, 1, 50,  80, 150;
        43200,  0, 0, 1, 50,  80, 150;
        64800,  2, 0, 0, 400, 80, 20;
        79200,  2, 1, 0, 150, 280, 20;
        82800,  2, 0, 0, 30,  80, 20;
        86400,  2, 0, 0, 30,  80, 20],
        // time[s], nPers_liv, nPers_cel, nPers_roof, P_liv[W], P_cel[W], P_roof[W]
                                           // Night
                                           // 6 AM: wake up
                                           // 8 AM: work (some at home)
                                           // 12 PM: lunch
                                           // 6 PM: everyone home
                                           // 10 PM: evening
                                           // 11 PM: bedtime
      columns = {2,3,4,5,6,7},
      extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic,
      smoothness = Modelica.Blocks.Types.Smoothness.ConstantSegments)
      "Occupancy schedule"
      annotation(Placement(transformation(extent={{-180,60},{-160,80}})));

    // ============================================================================
    // DHW DEMAND PROFILE (higher for multi-family)
    // ============================================================================
    Modelica.Blocks.Sources.CombiTimeTable dhwDemand(
      tableOnFile = false,
      table = [
        0,      0;
        21600,  80;
        25200,  50;
        28800,  20;
        43200,  30;
        64800,  70;
        72000,  60;
        79200,  30;
        82800,  10;
        86400,  0],
        // time[s], DHW_demand[L/h]
                          // Night
                          // 6 AM: morning peak
                          // 7 AM
                          // 8 AM
                          // 12 PM: lunch
                          // 6 PM: evening
                          // 8 PM
                          // 10 PM
                          // 11 PM
      columns = {2},
      extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic,
      smoothness = Modelica.Blocks.Types.Smoothness.ConstantSegments)
      "DHW demand profile [L/h]"
      annotation(Placement(transformation(extent={{-180,20},{-160,40}})));

    // ============================================================================
    // DHW STORAGE STATE
    // ============================================================================
    Modelica.Units.SI.Temperature T_DHW(start=T_DHW_init, fixed=true)
      "DHW storage temperature [K]";
    Real T_DHW_degC "DHW temperature [°C]";

    // ============================================================================
    // CONTROL SIGNALS (with hysteresis using pre())
    // ============================================================================
    Boolean CHP_on(start=true, fixed=true) "CHP on/off";
    Boolean CB_on(start=false, fixed=true) "CB on/off";
    Boolean DHW_pump_on(start=false, fixed=true) "DHW charging pump on/off";

    Real Mod_CHP "CHP modulation [0-1]";
    Real Mod_CB "CB modulation [0-1]";
    Real tempError "Temperature error [K]";

    // ============================================================================
    // INTERNAL VARIABLES
    // ============================================================================
    Real T_ambient_degC "Ambient temperature [°C]";
    Real T_ambient_K "Ambient temperature [K]";
    Real T_living_K "Living zone temp [K]";

    // Heat flows [W]
    Real Q_CHP "CHP thermal power [W]";
    Real Q_CB "CB thermal power [W]";
    Real Q_total "Total heat [W]";
    Real Q_to_DHW "Heat to DHW [W]";
    Real Q_loss_DHW "DHW losses [W]";
    Real Q_DHW_demand "DHW consumption [W]";

    // Temperatures
    Real T_supply_mixed_degC "Mixed supply temp [°C]";
    Real T_return_degC "Return from building [°C]";
    Real T_chp_supply "CHP supply temp [°C]";
    Real T_cb_supply "CB supply temp [°C]";

    // Flow rates
    Real qv_flow "Heating flow [L/min]";

    // ============================================================================
    // MONITORING OUTPUTS [kW]
    // ============================================================================
    Real Q_CHP_kW "CHP thermal [kW]";
    Real Q_CB_kW "CB thermal [kW]";
    Real Q_total_kW "Total heating [kW]";
    Real P_CHP_elec_kW "CHP electrical [kW]";
    Real Q_DHW_kW "DHW demand [kW]";

    // Temperatures [°C]
    Real T_living_degC "Living zone [°C]";
    Real T_cellar_degC "Cellar zone [°C]";
    Real T_roof_degC "Roof zone [°C]";

    // Modulation
    Real Modulation_pct "Modulation [%]";

    // ============================================================================
    // CUMULATIVE ENERGY [kWh]
    // ============================================================================
    Real E_CHP_th_kWh(start=0, fixed=true) "CHP thermal energy";
    Real E_CHP_el_kWh(start=0, fixed=true) "CHP electrical energy";
    Real E_CB_kWh(start=0, fixed=true) "CB thermal energy";
    Real E_total_kWh(start=0, fixed=true) "Total heating energy";
    Real E_DHW_kWh(start=0, fixed=true) "DHW energy";

    // ============================================================================
    // COMFORT & RUNTIME
    // ============================================================================
    Boolean in_comfort "In comfort band (18-24°C)";
    Real comfort_violation_s(start=0, fixed=true) "Time outside comfort [s]";
    Real comfort_pct "Comfort percentage [%]";

    Real CHP_runtime_h(start=0, fixed=true) "CHP runtime [h]";
    Real CB_runtime_h(start=0, fixed=true) "CB runtime [h]";
    Real CHP_runtime_pct, CB_runtime_pct "Runtime [%]";

    // Time
    Real time_hours "Time [h]";
    Real time_days "Time [days]";

    // ============================================================================
    // PHiL OUTPUTS
    // ============================================================================
    Modelica.Blocks.Interfaces.RealOutput T_roomIs_out = building.T_roomIs_degC
      "Living zone [°C]" annotation(Placement(transformation(extent={{180,60},{200,80}})));
    Modelica.Blocks.Interfaces.RealOutput T_cellarIs_out = building.T_cellarIs_degC
      "Cellar [°C]" annotation(Placement(transformation(extent={{180,30},{200,50}})));
    Modelica.Blocks.Interfaces.RealOutput T_roofIs_out = building.T_roofIs_degC
      "Roof [°C]" annotation(Placement(transformation(extent={{180,0},{200,20}})));
    Modelica.Blocks.Interfaces.RealOutput Q_CHP_kW_out = chp.QHeat_kW
      "CHP thermal [kW]" annotation(Placement(transformation(extent={{180,-30},{200,-10}})));
    Modelica.Blocks.Interfaces.RealOutput P_CHP_elec_kW_out = chp.PElec_kW
      "CHP electrical [kW]" annotation(Placement(transformation(extent={{180,-60},{200,-40}})));

  equation
    // ============================================================================
    // WEATHER DATA EXTRACTION
    // ============================================================================
    T_ambient_degC = weatherData.y[1];
    T_ambient_K = T_ambient_degC + 273.15;

    // ============================================================================
    // BUILDING INPUTS - Occupancy
    // ============================================================================
    building.nPersons_living_in = occupancyData.y[1];
    building.nPersons_cellar_in = occupancyData.y[2];
    building.nPersons_roof_in = occupancyData.y[3];
    building.P_appliances_living_W_in = occupancyData.y[4];
    building.P_appliances_cellar_W_in = occupancyData.y[5];
    building.P_appliances_roof_W_in = occupancyData.y[6];
    building.T_ambient_degC = T_ambient_degC;

    // ============================================================================
    // GET BUILDING TEMPERATURE FOR CONTROL
    // ============================================================================
    T_living_K = building.T_roomIs_degC + 273.15;
    T_living_degC = building.T_roomIs_degC;
    T_cellar_degC = building.T_cellarIs_degC;
    T_roof_degC = building.T_roofIs_degC;

    // ============================================================================
    // CONTROL - CHP (PRIMARY, hysteresis on room temp)
    // Turn ON when T_living < T_CHP_on, stay ON until T_living > T_CHP_off
    // ============================================================================
    CHP_on = (T_living_K < T_CHP_on) or (pre(CHP_on) and T_living_K < T_CHP_off);

    // ============================================================================
    // CONTROL - CB (BACKUP, only in emergency)
    // ============================================================================
    CB_on = (T_living_K < T_CB_on) or (pre(CB_on) and T_living_K < T_CB_off);

    // ============================================================================
    // MODULATION CALCULATION (proportional to error)
    // ============================================================================
    tempError = T_setpoint - T_living_K;
    Mod_CHP = noEvent(if CHP_on then max(0.3, min(1.0, tempError * 0.15)) else 0);
    Mod_CB = noEvent(if CB_on then max(0.2, min(1.0, tempError * 0.15)) else 0);
    Modulation_pct = Mod_CHP * 100;

    // ============================================================================
    // CONTROL - DHW PUMP (hysteresis)
    // ============================================================================
    DHW_pump_on = (T_DHW < T_DHW_pump_on) or
                  (pre(DHW_pump_on) and T_DHW < T_DHW_pump_off);

    // ============================================================================
    // RETURN TEMPERATURE FROM BUILDING
    // ============================================================================
    T_return_degC = building.STM_HCRL_Set_degC;
    qv_flow = 8;  // Fixed flow rate

    // ============================================================================
    // CHP CONNECTIONS
    // ============================================================================
    chp.CHPon = CHP_on;
    chp.Modulation = Mod_CHP;
    chp.TReturn_degC = T_return_degC;
    chp.qv_l_per_min = noEvent(if CHP_on then qv_flow else 0);
    T_chp_supply = chp.TSupply_degC;
    Q_CHP = chp.QHeat_kW * 1000;

    // ============================================================================
    // CB CONNECTIONS
    // ============================================================================
    cb.CBon = CB_on;
    cb.Modulation = Mod_CB;
    cb.TReturn_degC = T_return_degC;
    cb.qv_l_per_min = noEvent(if CB_on then qv_flow else 0);
    T_cb_supply = cb.TSupply_degC;
    Q_CB = cb.QHeat_kW * 1000;

    // ============================================================================
    // TOTAL HEAT AND MIXING
    // ============================================================================
    Q_total = Q_CHP + Q_CB;
    Q_total_kW = Q_total / 1000;

    // Mixed supply temperature (weighted by heat output)
    T_supply_mixed_degC = noEvent(if Q_total_kW > 0.01 then
      (chp.QHeat_kW * T_chp_supply + cb.QHeat_kW * T_cb_supply) / Q_total_kW
    else
      T_return_degC + 5);

    // ============================================================================
    // BUILDING SUPPLY
    // ============================================================================
    building.STM_HCVLaM_degC = T_supply_mixed_degC;
    building.SFW_HCRLbM_l_per_min = qv_flow;

    // ============================================================================
    // DHW STORAGE - Heat from main circuit (simplified)
    // ============================================================================
    Q_to_DHW = noEvent(if DHW_pump_on and (T_supply_mixed_degC + 273.15 > T_DHW + 5) then
                 min(3000, (T_supply_mixed_degC + 273.15 - T_DHW) * 200)
               else 0);
    Q_loss_DHW = UA_DHW * (T_DHW - T_ambient_K - 15);
    Q_DHW_demand = dhw.Q_DHW_W;

    C_DHW * der(T_DHW) = Q_to_DHW - Q_DHW_demand - Q_loss_DHW;
    T_DHW_degC = T_DHW - 273.15;

    // ============================================================================
    // DHW COMPONENT CONNECTIONS
    // ============================================================================
    dhw.qv_DHW_demand_Lph = dhwDemand.y[1];
    dhw.TStorage_degC = T_DHW_degC;
    dhw.TColdWater_degC = 10;

    // ============================================================================
    // MONITORING - kW
    // ============================================================================
    Q_CHP_kW = chp.QHeat_kW;
    Q_CB_kW = cb.QHeat_kW;
    P_CHP_elec_kW = chp.PElec_kW;
    Q_DHW_kW = Q_DHW_demand / 1000;

    // ============================================================================
    // ENERGY INTEGRATION
    // ============================================================================
    der(E_CHP_th_kWh) = Q_CHP_kW / 3600;
    der(E_CHP_el_kWh) = P_CHP_elec_kW / 3600;
    der(E_CB_kWh) = Q_CB_kW / 3600;
    der(E_total_kWh) = Q_total_kW / 3600;
    der(E_DHW_kWh) = Q_DHW_kW / 3600;

    // ============================================================================
    // COMFORT MONITORING
    // ============================================================================
    in_comfort = (T_living_degC >= 18) and (T_living_degC <= 24);
    der(comfort_violation_s) = if in_comfort then 0 else 1;
    comfort_pct = noEvent(if time > 0 then (1 - comfort_violation_s/time) * 100 else 100);

    // ============================================================================
    // RUNTIME TRACKING
    // ============================================================================
    der(CHP_runtime_h) = if CHP_on then 1/3600 else 0;
    der(CB_runtime_h) = if CB_on then 1/3600 else 0;

    CHP_runtime_pct = noEvent(if time > 0 then CHP_runtime_h / (time/3600) * 100 else 0);
    CB_runtime_pct = noEvent(if time > 0 then CB_runtime_h / (time/3600) * 100 else 0);

    // ============================================================================
    // TIME
    // ============================================================================
    time_hours = time / 3600;
    time_days = time / 86400;

    annotation(
      experiment(StartTime=0, StopTime=864000, Tolerance=1e-06, Interval=60),
      Documentation(info="<html>
<h4>MF5_Dymola - Complete Component-Based Model</h4>
<p>Multi-Family House configuration (no solar thermal):</p>
<ul>
<li><b>ThreeZoneBuilding_PHiL</b> - 240 m² building</li>
<li><b>SimpleCHP_v2</b> - Primary (5kW el / 12.5kW th)</li>
<li><b>SimpleCondensingBoiler</b> - Backup (50kW)</li>
<li><b>SimpleDHW</b> - DHW demand</li>
<li><b>DHW Storage</b> - 300L (simplified)</li>
</ul>

<h5>Control Strategy:</h5>
<ul>
<li><b>CHP</b>: PRIMARY - ON when T_living < 18°C, OFF when > 23°C (5K hysteresis)</li>
<li><b>CB</b>: BACKUP - ON when T_living < 14°C (emergency only)</li>
<li><b>DHW Pump</b>: ON when T_DHW < 45°C</li>
</ul>

<h5>Key Variables:</h5>
<p>Q_CHP_kW, Q_CB_kW, P_CHP_elec_kW, T_living_degC, Modulation_pct</p>
</html>"),
      Icon(graphics={
        Rectangle(extent={{-200,200},{200,-200}}, lineColor={0,0,0},
          fillColor={255,255,255}, fillPattern=FillPattern.Solid),
        Text(extent={{-180,190},{180,150}}, textColor={0,0,255},
          textString="MF5 Complete"),
        Text(extent={{-180,140},{180,110}}, textColor={128,128,128},
          textString="(No Solar)"),
        Rectangle(extent={{-140,60},{-60,0}}, lineColor={255,128,0},
          fillColor={255,200,150}, fillPattern=FillPattern.Solid),
        Text(extent={{-130,45},{-70,15}}, textString="CHP"),
        Text(extent={{-130,20},{-70,5}}, textString="12.5 kW"),
        Rectangle(extent={{-140,-20},{-60,-80}}, lineColor={0,0,255},
          fillColor={200,200,255}, fillPattern=FillPattern.Solid),
        Text(extent={{-130,-35},{-70,-55}}, textString="CB"),
        Text(extent={{-130,-55},{-70,-75}}, textString="50 kW"),
        Ellipse(extent={{-40,-40},{20,-55}}, lineColor={0,100,255},
          fillColor={150,200,255}, fillPattern=FillPattern.Solid),
        Rectangle(extent={{-40,-47},{20,-100}}, lineColor={0,100,255},
          fillColor={150,200,255}, fillPattern=FillPattern.Solid),
        Text(extent={{-35,-60},{15,-85}}, textString="300L"),
        Rectangle(extent={{60,80},{180,-60}}, lineColor={139,69,19},
          fillColor={255,228,196}, fillPattern=FillPattern.Solid),
        Polygon(points={{60,80},{120,130},{180,80},{60,80}},
          lineColor={178,34,34}, fillColor={178,34,34}, fillPattern=FillPattern.Solid),
        Text(extent={{70,60},{170,20}}, textString="Building"),
        Text(extent={{70,10},{170,-30}}, textString="240 m²"),
        Rectangle(extent={{60,-70},{100,-100}}, lineColor={0,100,255},
          fillColor={200,230,255}, fillPattern=FillPattern.Solid),
        Text(extent={{65,-75},{95,-95}}, textString="DHW"),
        Line(points={{-60,30},{60,30}}, color={255,0,0}, thickness=1),
        Line(points={{-60,-50},{-40,-70}}, color={0,100,255}, thickness=1),
        Line(points={{20,-70},{60,-80}}, color={0,100,255}, thickness=1)}

        // CHP

        // CB

        // DHW

        // Building

        // DHW tap

        // Connections
));
  end MF5_Dymola;

end Scenarios;
