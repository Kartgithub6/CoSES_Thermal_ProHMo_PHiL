within CoSES_Thermal_ProHMo_PHiL.Backup;
model TestBuilding_CHP_CB_v2
  "Integrated CHP + CB + Building test model - FIXED VERSION"

  // ============================================================================
  // HEAT GENERATORS (using v2 models with verified physics)
  // ============================================================================

  Backup.SimpleCHP_v2 chp(
    QHeat_nominal=12500,
    // 12.5 kW thermal
    PElec_nominal=5000,
    // 5 kW electrical
    tau=60) "Combined Heat and Power unit"
    annotation (Placement(transformation(extent={{-136,60},{-96,100}})));

  Backup.SimpleCondensingBoiler cb(
    QHeat_nominal=50000,
    // 50 kW thermal (backup)
    eta_nominal=0.98,
    eta_noncondensing=0.88,
    ModulationMin=0.20,
    tau=30) "Condensing Boiler - backup heat source"
    annotation (Placement(transformation(extent={{-160,-20},{-120,20}})));

  // ============================================================================
  // THE BUILDING
  // ============================================================================

  ThermoEnergeticAnalysis.ThreeZoneBuilding_PHiL building(
    AZone_cellar=80,
    AZone_living=100,
    AZone_roof=60,
    TZoneInit_cellar=288.15,
    // 15°C
    TZoneInit_living=291.15,
    // 18°C (start below setpoint)
    TZoneInit_roof=289.15,
    // 16°C
    TRef_cellar=288.15,
    // 15°C setpoint
    TRef_living=294.15,
    // 21°C setpoint
    TRef_roof=294.15,
    // 21°C setpoint
    cellarHeat=true,
    roofHeat=true)
    annotation (Placement(transformation(extent={{60,-50},{140,50}})));

  // ============================================================================
  // CONTROL PARAMETERS
  // ============================================================================

  parameter Real T_CHP_on = 293.15 "CHP ON below 20°C [K]";
  parameter Real T_CHP_off = 295.15 "CHP OFF above 22°C [K]";
  parameter Real T_CB_on = 291.15 "CB ON below 18°C [K] (emergency backup)";
  parameter Real T_CB_off = 293.15 "CB OFF above 20°C [K]";
  parameter Real qv_nominal = 6 "Flow rate [L/min]";

  // ============================================================================
  // ENVIRONMENT
  // ============================================================================

  Modelica.Blocks.Sources.Sine ambientTemp(
    amplitude = 3,
    f = 1/86400,
    offset = 5,
    phase = -1.5708)
    "Daily outdoor temperature 2-8°C"
    annotation(Placement(transformation(extent={{-60,-80},{-40,-60}})));

  // ============================================================================
  // INTERNAL GAINS (Constant)
  // ============================================================================

  Modelica.Blocks.Sources.Constant nPersons_living(k=2)
    annotation(Placement(transformation(extent={{-20,30},{0,50}})));
  Modelica.Blocks.Sources.Constant nPersons_cellar(k=0)
    annotation(Placement(transformation(extent={{-20,0},{0,20}})));
  Modelica.Blocks.Sources.Constant nPersons_roof(k=0)
    annotation(Placement(transformation(extent={{-20,-30},{0,-10}})));
  Modelica.Blocks.Sources.Constant P_app_living(k=200)
    annotation(Placement(transformation(extent={{180,20},{160,40}})));
  Modelica.Blocks.Sources.Constant P_app_cellar(k=80)
    annotation(Placement(transformation(extent={{180,-10},{160,10}})));
  Modelica.Blocks.Sources.Constant P_app_roof(k=50)
    annotation(Placement(transformation(extent={{180,-40},{160,-20}})));

  // ============================================================================
  // FLOW RATE
  // ============================================================================

  Modelica.Blocks.Sources.Constant flowRate(k=qv_nominal)
    annotation(Placement(transformation(extent={{-220,30},{-200,50}})));

  // ============================================================================
  // CONTROL - CHP Hysteresis
  // ============================================================================

  Modelica.Blocks.Logical.Hysteresis CHP_hysteresis(
    uLow = T_CHP_on,
    uHigh = T_CHP_off,
    pre_y_start = true)
    annotation(Placement(transformation(extent={{-220,130},{-200,150}})));

  Modelica.Blocks.Logical.Not CHP_on_signal
    annotation(Placement(transformation(extent={{-190,130},{-170,150}})));

  // ============================================================================
  // CONTROL - CB Hysteresis (backup)
  // ============================================================================

  Modelica.Blocks.Logical.Hysteresis CB_hysteresis(
    uLow = T_CB_on,
    uHigh = T_CB_off,
    pre_y_start = false)
    annotation(Placement(transformation(extent={{-220,90},{-200,110}})));

  Modelica.Blocks.Logical.Not CB_on_signal
    annotation(Placement(transformation(extent={{-190,90},{-170,110}})));

  // ============================================================================
  // MODULATION CONTROL
  // ============================================================================

  Modelica.Blocks.Math.Add tempError(k1=1, k2=-1)
    "Error = Setpoint - Actual (positive when heating needed)"
    annotation(Placement(transformation(extent={{-220,60},{-200,80}})));

  Modelica.Blocks.Sources.Constant T_setpoint_K(k=294.15)
    "21°C setpoint in Kelvin"
    annotation(Placement(transformation(extent={{-260,64},{-240,84}})));

  Modelica.Blocks.Math.Gain modulationGain(k=0.2)
    "5K error = 100% modulation"
    annotation(Placement(transformation(extent={{-190,60},{-170,80}})));

  Modelica.Blocks.Nonlinear.Limiter modulationLimiter(uMax=1, uMin=0)
    annotation(Placement(transformation(extent={{-154,36},{-134,56}})));

  // ============================================================================
  // OUTPUT VARIABLES FOR PLOTTING
  // ============================================================================

  // Temperatures
  Real T_living_degC = building.T_roomIs_degC "Living zone [°C]";
  Real T_cellar_degC = building.T_cellarIs_degC "Cellar [°C]";
  Real T_roof_degC = building.T_roofIs_degC "Roof [°C]";
  Real T_supply_degC "Supply to building [°C]";
  Real T_return_degC = building.STM_HCRL_Set_degC "Return from building [°C]";
  Real T_ambient_degC = ambientTemp.y "Outdoor [°C]";

  // Heat output
  Real CHP_QHeat_kW = chp.QHeat_kW "CHP thermal [kW]";
  Real CB_QHeat_kW = cb.QHeat_kW "CB thermal [kW]";
  Real Total_QHeat_kW = chp.QHeat_kW + cb.QHeat_kW "Total heat [kW]";

  // Electrical
  Real CHP_PElec_kW = chp.PElec_kW "CHP electrical [kW]";

  // Control
  Real CHP_on = if CHP_on_signal.y then 1 else 0 "CHP status (1=ON)";
  Real CB_on = if CB_on_signal.y then 1 else 0 "CB status (1=ON)";
  Real Modulation_pct = modulationLimiter.y * 100 "Modulation [%]";

  // Time
  Real time_hours = time/3600;
  Real time_days = time/86400;

  // Debug - CHP internals
  Real CHP_deltaT = chp.debug_deltaT_K "CHP temperature rise [K]";
  Real CHP_m_flow = chp.debug_m_flow_kg_s "CHP mass flow [kg/s]";

equation
  // ============================================================================
  // CONTROL LOGIC
  // ============================================================================

  // Convert living temp to Kelvin for hysteresis
  CHP_hysteresis.u = building.T_roomIs_degC + 273.15;
  CB_hysteresis.u = building.T_roomIs_degC + 273.15;

  // Temperature error for modulation
  connect(T_setpoint_K.y, tempError.u1);
  tempError.u2 = building.T_roomIs_degC + 273.15;

  // Hysteresis to on/off signals
  connect(CHP_hysteresis.y, CHP_on_signal.u);
  connect(CB_hysteresis.y, CB_on_signal.u);

  // Modulation chain
  connect(tempError.y, modulationGain.u);
  connect(modulationGain.y, modulationLimiter.u);

  // ============================================================================
  // CHP CONNECTIONS
  // ============================================================================

  connect(CHP_on_signal.y, chp.CHPon);
  connect(modulationLimiter.y, chp.Modulation);
  chp.TReturn_degC = building.STM_HCRL_Set_degC;
  connect(flowRate.y, chp.qv_l_per_min);

  // ============================================================================
  // CB CONNECTIONS
  // ============================================================================

  connect(CB_on_signal.y, cb.CBon);
  connect(modulationLimiter.y, cb.Modulation);
  cb.TReturn_degC = building.STM_HCRL_Set_degC;
  connect(flowRate.y, cb.qv_l_per_min);

  // ============================================================================
  // SUPPLY TEMPERATURE TO BUILDING
  // ============================================================================
  // Use CHP output directly (CB adds to it when running)
  // Weighted average when both are running

  if (chp.QHeat_kW + cb.QHeat_kW) > 0.1 then
    T_supply_degC = (chp.QHeat_kW * chp.TSupply_degC + cb.QHeat_kW * cb.TSupply_degC)
                    / (chp.QHeat_kW + cb.QHeat_kW);
  else
    T_supply_degC = building.STM_HCRL_Set_degC;  // No heating: supply = return
  end if;

  building.STM_HCVLaM_degC = T_supply_degC;

  // ============================================================================
  // OTHER BUILDING CONNECTIONS
  // ============================================================================

  connect(flowRate.y, building.SFW_HCRLbM_l_per_min);
  connect(ambientTemp.y, building.T_ambient_degC);

  connect(nPersons_living.y, building.nPersons_living_in);
  connect(nPersons_cellar.y, building.nPersons_cellar_in);
  connect(nPersons_roof.y, building.nPersons_roof_in);
  connect(P_app_living.y, building.P_appliances_living_W_in);
  connect(P_app_cellar.y, building.P_appliances_cellar_W_in);
  connect(P_app_roof.y, building.P_appliances_roof_W_in);

  annotation(
    experiment(StopTime=259200, Interval=60, Tolerance=1e-06),
    Diagram(coordinateSystem(extent={{-280,-100},{200,180}})),
    Documentation(info="<html>
<h4>TestBuilding_CHP_CB_v2 - Fixed Integrated Model</h4>

<h5>Key Variables to Plot:</h5>
<ul>
<li><b>T_living_degC</b> - Should rise toward 21°C</li>
<li><b>T_supply_degC</b> - Should be ~55-65°C when heating</li>
<li><b>T_return_degC</b> - Should be ~35-45°C</li>
<li><b>CHP_QHeat_kW</b> - Heat from CHP (up to 12.5 kW)</li>
<li><b>CB_QHeat_kW</b> - Heat from CB (backup, up to 50 kW)</li>
<li><b>CHP_PElec_kW</b> - Electricity generated (up to 5 kW)</li>
<li><b>CHP_on, CB_on</b> - Control signals (0 or 1)</li>
</ul>

<h5>Debug Variables:</h5>
<ul>
<li><b>CHP_deltaT</b> - Should be ~30K at full power</li>
<li><b>CHP_m_flow</b> - Should be 0.1 kg/s</li>
</ul>

<h5>Control Logic:</h5>
<ul>
<li>CHP: ON when T_living < 20°C, OFF when > 22°C</li>
<li>CB: ON when T_living < 18°C, OFF when > 20°C (backup only)</li>
</ul>
</html>"));
end TestBuilding_CHP_CB_v2;
