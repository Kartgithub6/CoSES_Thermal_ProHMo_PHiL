within CoSES_Thermal_ProHMo_PHiL.HydronicSystem;
model TestHydronicSystem
  "Test setup for SystemWithZoneAndHydronics and WaterToHeatPort"

  replaceable package Medium = IBPSA.Media.Water annotation(choicesAllMatching=true);
  inner Modelica.Fluid.System system;


  // Heat source supplying hot water
  IBPSA.Fluid.Sources.MassFlowSource_T hotSource(
    redeclare package Medium = Medium,
    T = 333.15,          // 60°C supply temp
    m_flow = 0.2,        // kg/s
    nPorts = 1,
    use_m_flow_in = false)
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));


  // Sink to receive return water
  Modelica.Fluid.Sources.Boundary_pT coldSink(
    redeclare package Medium = Medium,
    T=293.15,      // 20°C
    p=system.p_ambient,
    nPorts=1)
    annotation (Placement(transformation(extent={{100,-10},{80,10}})));

  // Component under test
  SystemWithZoneAndHydronics sysUT(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

  // Dummy thermal zone
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor zone(
    C=1e6, T(start=293.15))
    annotation (Placement(transformation(extent={{-10,60},{10,80}})));

equation


  connect(hotSource.ports[1], sysUT.port_a)
    annotation (Line(points={{-80,0},{-9,0}}, color={0,127,255}));
  connect(sysUT.port_b, coldSink.ports[1])
    annotation (Line(points={{9,0},{80,0}}, color={0,127,255}));
  connect(zone.port, sysUT.zoneHeatPort)
    annotation (Line(points={{0,60},{0,9}}, color={191,0,0}));
annotation(
  Diagram(coordinateSystem(extent={{-100,-100},{100,100}})), experiment(
        StopTime=2000, __Dymola_Algorithm="Dassl"));
end TestHydronicSystem;
