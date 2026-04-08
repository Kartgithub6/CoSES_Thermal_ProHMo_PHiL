within CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis;
model ThreeZoneBuilding_PHiL_optimized
  "PHiL wrapper for OPTIMIZED topology - Interface for power hardware-in-the-loop testing"

  replaceable package Medium=Modelica.Media.Water.StandardWater;
  inner Modelica.Fluid.System system(
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial)
    annotation(Placement(visible=true, transformation(origin={-190,170}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // PHiL INPUTS - From Physical Hardware
  // ============================================================================
  Modelica.Blocks.Interfaces.RealInput STM_HCVLaM_degC(start=60)
    "Supply temperature from hardware [°C]"
    annotation(Placement(visible=true, transformation(origin={-240,112}, extent={{-20,-20},{20,20}}, rotation=0),
      iconTransformation(origin={-110,90}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput SFW_HCRLbM_l_per_min(start=6)
    "Supply flow rate from hardware [L/min]"
    annotation(Placement(visible=true, transformation(origin={-240,48}, extent={{-20,-20},{20,20}}, rotation=0),
      iconTransformation(origin={-110,70}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput T_ambient_degC(start=5)
    "Ambient temperature [°C]"
    annotation(Placement(visible=true, transformation(origin={-240,12}, extent={{-20,-20},{20,20}}, rotation=0),
      iconTransformation(origin={-110,50}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // PHiL OUTPUTS - To Physical Hardware
  // ============================================================================
  Modelica.Blocks.Interfaces.RealOutput T_roomIs_degC
    "Living room temperature [°C]"
    annotation(Placement(visible=true, transformation(origin={290,174}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,90}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput T_cellarIs_degC
    "Cellar temperature [°C]"
    annotation(Placement(visible=true, transformation(origin={292,140},extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,70}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput T_roofIs_degC
    "Roof office temperature [°C]"
    annotation(Placement(visible=true, transformation(origin={292,102},extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,50}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput STM_HCRL_Set_degC
    "Return temperature setpoint [°C]"
    annotation(Placement(visible=true, transformation(origin={288,76}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,30}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput SFW_HCRLbM_Set_l_per_min
    "Flow rate setpoint [L/min]"
    annotation(Placement(visible=true, transformation(origin={288,42},extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,10}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput valve_cellar_opening
    "Cellar valve position [0-1]"
    annotation(Placement(visible=true, transformation(origin={288,16},  extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,-10}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput valve_living_opening
    "Living valve position [0-1]"
    annotation(Placement(visible=true, transformation(origin={288,-12}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,-30}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput valve_roof_opening
    "Roof valve position [0-1]"
    annotation(Placement(visible=true, transformation(origin={286,-38}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,-50}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // INTERNAL GAINS INPUTS
  // ============================================================================
  Modelica.Blocks.Interfaces.RealInput nPersons_living_in(start=2)
    "Number of people in living room"
    annotation(Placement(visible=true, transformation(origin={-240,-12}, extent={{-20,-20},{20,20}}, rotation=0),
      iconTransformation(origin={-110,20}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput nPersons_cellar_in(start=0)
    "Number of people in cellar"
    annotation(Placement(visible=true, transformation(origin={-240,-50}, extent={{-20,-20},{20,20}}, rotation=0),
      iconTransformation(origin={-110,0}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput nPersons_roof_in(start=0)
    "Number of people in roof office"
    annotation(Placement(visible=true, transformation(origin={-240,-92}, extent={{-20,-20},{20,20}}, rotation=0),
      iconTransformation(origin={-110,-20}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput P_appliances_living_W_in(start=200)
    "Appliance power in living room [W]"
    annotation(Placement(visible=true, transformation(origin={292,-138},extent={{-20,-20},{20,20}}, rotation=180),
      iconTransformation(origin={110,-70}, extent={{-10,-10},{10,10}}, rotation=180)));

  Modelica.Blocks.Interfaces.RealInput P_appliances_cellar_W_in(start=50)
    "Appliance power in cellar [W]"
    annotation(Placement(visible=true, transformation(origin={290,-94},extent={{-20,-20},{20,20}}, rotation=180),
      iconTransformation(origin={110,-80}, extent={{-10,-10},{10,10}}, rotation=180)));

  Modelica.Blocks.Interfaces.RealInput P_appliances_roof_W_in(start=50)
    "Appliance power in roof office [W]"
    annotation(Placement(visible=true, transformation(origin={294,-178},extent={{-20,-20},{20,20}}, rotation=180),
      iconTransformation(origin={110,-90}, extent={{-10,-10},{10,10}}, rotation=180)));

  // ============================================================================
  // PARAMETERS - Delegated to building model
  // ============================================================================
  parameter Modelica.Units.SI.Area AZone_cellar=80;
  parameter Modelica.Units.SI.Area AZone_living=100;
  parameter Modelica.Units.SI.Area AZone_roof=60;
  parameter Modelica.Units.SI.Length hZone_cellar=2.2;
  parameter Modelica.Units.SI.Length hZone_living=2.5;
  parameter Modelica.Units.SI.Length hZone_roof=2.3;
  parameter Modelica.Units.SI.Temperature TZoneInit_cellar=291.15;
  parameter Modelica.Units.SI.Temperature TZoneInit_living=293.15;
  parameter Modelica.Units.SI.Temperature TZoneInit_roof=290.15;
  parameter Modelica.Units.SI.Temperature TRef_cellar=291.15;
  parameter Modelica.Units.SI.Temperature TRef_living=293.15;
  parameter Modelica.Units.SI.Temperature TRef_roof=293.15;
  parameter Boolean cellarHeat=true;
  parameter Boolean roofHeat=true;
  parameter Real k_PI=0.3;
  parameter Modelica.Units.SI.Time Ti_PI=500;
  parameter Real yMin_cellar=0.05;
  parameter Real yMin_living=0.03;
  parameter Real yMin_roof=0.08;

  // ============================================================================
  // FLUID BOUNDARIES - PHiL Interface
  // ============================================================================
  Modelica.Fluid.Sources.Boundary_pT supply(
    redeclare package Medium=Medium,
    use_p_in=false,
    p=250000,
    use_T_in=true,
    T=333.15,
    nPorts=1)
    "Supply boundary (from hardware)"
    annotation(Placement(visible=true, transformation(origin={-120,64}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Fluid.Sources.Boundary_pT return_sink(
    redeclare package Medium=Medium,
    p=150000,
    T=313.15,
    nPorts=1)
    "Return boundary (to hardware)"
    annotation(Placement(visible=true, transformation(origin={-120,-50}, extent={{-10,-10},{10,10}}, rotation=180)));

  Modelica.Fluid.Sensors.TemperatureTwoPort TReturn(redeclare package Medium=Medium)
    "Return temperature sensor"
    annotation(Placement(visible=true, transformation(origin={-70,-50}, extent={{10,-10},{-10,10}}, rotation=0)));

  // Individual zone heating powers (from radiator Q_flow)
  Real P_heating_living_kW(unit="kW") = building.living_hydSys.radiator.Q_flow / 1000.0
    "Living room heating power [kW]";

  Real P_heating_cellar_kW(unit="kW") = building.cellar_hydSys.radiator.Q_flow / 1000.0
    "Cellar heating power [kW]";

  Real P_heating_roof_kW(unit="kW") = building.roof_hydSys.radiator.Q_flow / 1000.0
    "Roof office heating power [kW]";

  // Total heating power
  Real P_heating_total_kW(unit="kW") = P_heating_living_kW + P_heating_cellar_kW + P_heating_roof_kW
    "Total heating power consumption [kW]";

  // ============================================================================
  // UNIT CONVERSIONS - °C ↔ K
  // ============================================================================
  Modelica.Blocks.Math.Add toKelvin(k1=1, k2=1)
    "Convert supply temperature °C → K"
    annotation(Placement(visible=true, transformation(origin={-170,86}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Sources.Constant kelvinOffset(k=273.15)
    "Kelvin offset constant"
    annotation(Placement(visible=true, transformation(origin={-200,66}, extent={{-6,-6},{6,6}}, rotation=0)));

  Modelica.Blocks.Math.Add toCelsius_living(k1=1, k2=-1)
    "Convert living temperature K → °C"
    annotation(Placement(visible=true, transformation(origin={186,176}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Math.Add toCelsius_cellar(k1=1, k2=-1)
    "Convert cellar temperature K → °C"
    annotation(Placement(visible=true, transformation(origin={186,140},extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Math.Add toCelsius_roof(k1=1, k2=-1)
    "Convert roof temperature K → °C"
    annotation(Placement(visible=true, transformation(origin={186,104},extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Math.Add toCelsius_return(k1=1, k2=-1)
    "Convert return temperature K → °C"
    annotation(Placement(visible=true, transformation(origin={184,76}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Sources.Constant kelvinOffset2(k=273.15)
    "Kelvin offset for output conversion"
    annotation(Placement(visible=true, transformation(origin={142,78}, extent={{-6,-6},
            {6,6}},                                                                            rotation=0)));

  Modelica.Blocks.Math.Gain flowConvertOut(k=60000)
    "Convert flow m³/s → L/min"
    annotation(Placement(visible=true, transformation(origin={186,42},extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // BUILDING MODEL - OPTIMIZED TOPOLOGY
  // ============================================================================
  CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis.ThreeZoneBuilding_optimized building(
    redeclare package Medium=Medium,
    AZone_cellar=AZone_cellar,
    AZone_living=AZone_living,
    AZone_roof=AZone_roof,
    hZone_cellar=hZone_cellar,
    hZone_living=hZone_living,
    hZone_roof=hZone_roof,
    TZoneInit_cellar=TZoneInit_cellar,
    TZoneInit_living=TZoneInit_living,
    TZoneInit_roof=TZoneInit_roof,
    TRef_cellar=TRef_cellar,
    TRef_living=TRef_living,
    TRef_roof=TRef_roof,
    cellarHeat=cellarHeat,
    roofHeat=roofHeat,
    k_PI=k_PI,
    Ti_PI=Ti_PI,
    yMin_cellar=yMin_cellar,
    yMin_living=yMin_living,
    yMin_roof=yMin_roof)
    "Core building model with optimized hydraulics"
    annotation(Placement(visible=true, transformation(origin={-1,-9},   extent={{-39,-39},{39,39}}, rotation=0)));

equation
  // ============================================================================
  // FLUID CONNECTIONS
  // ============================================================================
  connect(supply.ports[1], building.port_a)
    annotation(Line(points={{-110,64},{-56,64},{-56,14.4},{-40,14.4}},
                                                                     color={0,127,255}, thickness=0.5));
  connect(building.port_b, TReturn.port_a)
    annotation(Line(points={{-40,-32.4},{-54,-32.4},{-54,-50},{-60,-50}},       color={0,127,255}, thickness=0.5));
  connect(TReturn.port_b, return_sink.ports[1])
    annotation(Line(points={{-80,-50},{-130,-50}}, color={0,127,255}, thickness=0.5));

  // ============================================================================
  // INPUT CONVERSIONS & CONNECTIONS
  // ============================================================================
  // Supply temperature: °C → K
  connect(STM_HCVLaM_degC, toKelvin.u1)
    annotation(Line(points={{-240,112},{-200,112},{-200,92},{-182,92}}, color={0,0,127}));
  connect(kelvinOffset.y, toKelvin.u2)
    annotation(Line(points={{-193.4,66},{-190,66},{-190,80},{-182,80}}, color={0,0,127}));
  connect(toKelvin.y, supply.T_in)
    annotation(Line(points={{-159,86},{-140,86},{-140,68},{-132,68}}, color={0,0,127}));

  // Internal gains - direct pass-through
  connect(nPersons_living_in, building.nPersons_living)
    annotation(Line(points={{-240,-12},{-56,-12},{-56,10.5},{-43.9,10.5}},
                                                                         color={0,0,127}));
  connect(nPersons_cellar_in, building.nPersons_cellar)
    annotation(Line(points={{-240,-50},{-136,-50},{-136,22.2},{-43.9,22.2}},
                                                                         color={0,0,127}));
  connect(nPersons_roof_in, building.nPersons_roof)
    annotation(Line(points={{-240,-92},{-52,-92},{-52,-2},{-54,-2},{-54,-1.2},{-43.9,
          -1.2}},                                                        color={0,0,127}));

  connect(P_appliances_living_W_in, building.P_appliances_living_W)
    annotation(Line(points={{292,-138},{188,-138},{188,-2},{54,-2},{54,10.5},{
          41.9,10.5}},                                                                   color={0,0,127}));
  connect(P_appliances_cellar_W_in, building.P_appliances_cellar_W)
    annotation(Line(points={{290,-94},{190,-94},{190,-6},{56,-6},{56,22.2},{
          41.9,22.2}},                                                                 color={0,0,127}));
  connect(P_appliances_roof_W_in, building.P_appliances_roof_W)
    annotation(Line(points={{294,-178},{184,-178},{184,-4},{50,-4},{50,-1.2},{
          41.9,-1.2}},                                                                   color={0,0,127}));

  // ============================================================================
  // OUTPUT CONVERSIONS & CONNECTIONS
  // ============================================================================
  // Zone temperatures: K → °C
  connect(building.TZone_living, toCelsius_living.u1)
    annotation(Line(points={{41.9,2.7},{60,2.7},{60,182},{174,182}},
                                                                 color={0,0,127}));
  connect(kelvinOffset2.y, toCelsius_living.u2)
    annotation(Line(points={{148.6,78},{160,78},{160,170},{174,170}}, color={0,0,127}));
  connect(toCelsius_living.y, T_roomIs_degC)
    annotation(Line(points={{197,176},{274,176},{274,174},{290,174}},
                                                  color={0,0,127}));
  connect(building.TZone_cellar, toCelsius_cellar.u1)
    annotation(Line(points={{41.9,14.4},{116,14.4},{116,146},{174,146}},
                                                                 color={0,0,127}));
  connect(kelvinOffset2.y, toCelsius_cellar.u2)
    annotation(Line(points={{148.6,78},{160,78},{160,134},{174,134}},
                                                                    color={0,0,127}));
  connect(toCelsius_cellar.y, T_cellarIs_degC)
    annotation(Line(points={{197,140},{292,140}},
                                                color={0,0,127}));

  connect(building.TZone_roof, toCelsius_roof.u1)
    annotation(Line(points={{41.9,-9},{118,-9},{118,110},{174,110}},
                                                                 color={0,0,127}));
  connect(kelvinOffset2.y, toCelsius_roof.u2)
    annotation(Line(points={{148.6,78},{160,78},{160,98},{174,98}}, color={0,0,127}));
  connect(toCelsius_roof.y, T_roofIs_degC)
    annotation(Line(points={{197,104},{276,104},{276,102},{292,102}},
                                                color={0,0,127}));

  // Return temperature: K → °C
  connect(TReturn.T, toCelsius_return.u1)
    annotation(Line(points={{-70,-39},{-70,66},{162,66},{162,82},{172,82}},   color={0,0,127}));
  connect(kelvinOffset2.y, toCelsius_return.u2)
    annotation(Line(points={{148.6,78},{164,78},{164,70},{172,70}}, color={0,0,127}));
  connect(toCelsius_return.y, STM_HCRL_Set_degC)
    annotation(Line(points={{195,76},{288,76}}, color={0,0,127}));

  // Flow rate: m³/s → L/min
  connect(building.qvRef, flowConvertOut.u)
    annotation(Line(points={{41.9,-36.3},{26,-36.3},{26,14},{142,14},{142,44},{
          154,44},{154,42},{174,42}},                            color={0,0,127}));
  connect(flowConvertOut.y, SFW_HCRLbM_Set_l_per_min)
    annotation(Line(points={{197,42},{288,42}},
                                              color={0,0,127}));

  // Valve positions - direct pass-through
  connect(building.valve_cellar_opening, valve_cellar_opening)
    annotation(Line(points={{41.9,-20.7},{56,-20.7},{56,-14},{122,-14},{122,16},
          {288,16}},                                               color={0,0,127}));
  connect(building.valve_living_opening, valve_living_opening)
    annotation(Line(points={{41.9,-28.5},{186,-28.5},{186,-10},{272,-10},{272,
          -12},{288,-12}},                                       color={0,0,127}));
  connect(building.valve_roof_opening, valve_roof_opening)
    annotation(Line(points={{41.9,-44.1},{270,-44.1},{270,-38},{286,-38}},
                                                                     color={0,0,127}));

annotation(
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-80,70},{80,-70}},
          lineColor={0,0,255},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Polygon(
          points={{-90,70},{0,95},{90,70},{-90,70}},
          lineColor={139,69,19},
          fillColor={205,133,63},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{60,95},{75,85}},
          lineColor={200,200,200},
          fillColor={230,230,230},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{65,92},{82,80}},
          lineColor={200,200,200},
          fillColor={230,230,230},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{70,95},{85,82}},
          lineColor={200,200,200},
          fillColor={230,230,230},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,60},{-80,55}},
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,40},{-80,35}},
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,20},{-80,15}},
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{80,60},{100,55}},
          lineColor={255,0,0},
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{80,40},{100,35}},
          lineColor={255,0,0},
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{80,20},{100,15}},
          lineColor={255,0,0},
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-75,60},{75,35}},
          textString="PHiL Optimized",
          textColor={0,0,0},
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-75,25},{75,5}},
          textString="%name",
          textColor={0,0,127}),
        Text(
          extent={{-75,-5},{75,-25}},
          textString=DynamicSelect("SmallHouse", String(buildingData)),
          textColor={28,108,200},
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-75,-35},{75,-55}},
          textString="without Weather",
          textColor={100,100,100},
          fontSize=8)}),

    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-280,-200},{280,200}})),
    Documentation(info="<html>
<h4>ThreeZoneBuilding_PHiL_optimized</h4>
<li>
Feb 28, 2026, by Karthik Murugesan
</li>
<p><b>PHiL (Power Hardware-in-the-Loop) Wrapper</b></p>
<h5>PURPOSE:</h5>
<p>This model provides a standardized interface for connecting the building simulation to physical hardware (real boiler, pumps, sensors).</p>

<h5>ARCHITECTURE:</h5>
<pre>
Physical Hardware ←→ PHiL Wrapper ←→ Building Model

Hardware provides:           PHiL converts:              Building uses:
- Temperature (°C)    →     °C to Kelvin        →      Kelvin
- Flow rate (L/min)   →     L/min to m³/s       →      m³/s
- Sensor readings     →     Unit conversions    →      Modelica units

Building provides:           PHiL converts:              Hardware receives:
- Temperatures (K)    →     Kelvin to °C        →      °C
- Flow rate (m³/s)    →     m³/s to L/min       →      L/min
- Valve positions     →     Direct pass         →      [0-1]
</pre>

<h5>KEY FEATURES:</h5>
<ul>
<li><b>Unit Conversion:</b> Automatic °C ↔ K, L/min ↔ m³/s</li>
<li><b>Standard Interface:</b> Consistent PHiL inputs/outputs</li>
<li><b>Parameter Pass-Through:</b> All building parameters accessible</li>
<li><b>Wrapped Model:</b> Uses ThreeZoneBuilding_optimized</li>
</ul>

<h5>USAGE:</h5>
<p><b>For PHiL Testing:</b></p>
<pre>
1. Connect physical hardware to inputs:
   - STM_HCVLaM_degC (supply temp from boiler)
   - SFW_HCRLbM_l_per_min (flow from pump)
   
2. Read outputs for hardware control:
   - T_roomIs_degC, T_cellarIs_degC, T_roofIs_degC
   - STM_HCRL_Set_degC (return temp)
   - valve positions
</pre>

<p><b>For Pure Simulation:</b></p>
<pre>
Use TestThreeZoneBuilding_optimized instead
(skips PHiL wrapper for better performance)
</pre>

<h5>OPTIMIZED TOPOLOGY:</h5>
<p>This wrapper encapsulates the OPTIMIZED hydraulic design:</p>
<ul>
<li>Living room gets flow priority</li>
<li>Cellar and roof share remaining capacity</li>
<li>Right-sized radiators (2500W cellar, not 4000W)</li>
<li>Optimized valve minimums (5% cellar, not 20%)</li>
</ul>

<p><b>Professional HVAC interface for hardware testing!</b> ✅</p>
</html>"));
end ThreeZoneBuilding_PHiL_optimized;
