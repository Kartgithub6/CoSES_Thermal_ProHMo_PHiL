within CoSES_Thermal_ProHMo_PHiL.BuildingSystem.Building;
model TestHydronicZone
  inner Modelica.Fluid.System system;
  replaceable package Medium = Modelica.Media.Water.StandardWater
    annotation(Placement(transformation(extent={{-100,80},{-60,100}})));

  // Hydronic system
  HydronicSystem.SystemWithZoneAndHydronics hydSys(
    redeclare package Medium=Medium     // changed from Modelica.Media.Water.StandardWater
)   annotation(Placement(transformation(extent={{-40,-20},{0,20}})));

  // Supply boundary (acts like boiler supply)
  Modelica.Fluid.Sources.Boundary_pT supply(
    redeclare package Medium = Medium,
      p = 300000,
      T = 333.15,
      nPorts = 1)
    annotation(Placement(transformation(extent={{-100,-10},{-80,10}})));

  // 13 Dec 17.00 Return boundary (boiler return) - used MassFlowSource to avoid pressure conflict
  Modelica.Fluid.Sources.MassFlowSource_T ret(
    redeclare package Medium = Medium,
      use_m_flow_in = false,
      m_flow = -0.05,
      T = 303.15,
      nPorts = 1)
    annotation(Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
      origin={10,-82})));

 /* // 13 Dec 17.00 commenteds
  // Return boundary (boiler return)
  Modelica.Fluid.Sources.Boundary_pT ret(
    redeclare package Medium = Medium, // IBPSA.Media.Water,
      p = 280000,
      T = 303.15,
      nPorts = 1)s
    annotation(Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={10,-82})));
 */


  // 13 Dec added TZoneRefConst & WindowShadingConst
  // Constant inputs for zone
  Modelica.Blocks.Sources.Constant TZoneRefConst(k=293.15)
    "Reference zone temperature 20°C"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={64,-12})));

  Modelica.Blocks.Sources.Constant WindowShadingConst[3](each k=0)
    "No window shading"
    annotation (Placement(transformation(extent={{16,-22},{36,-2}})));
  Modelica.Blocks.Sources.Constant TBoundOutConst[3](each k=283.15)
    annotation (Placement(transformation(extent={{-22,40},{-2,60}})));
  HeatedZone heatedZone
    annotation (Placement(transformation(extent={{48,18},{68,38}})));
equation
/* // 15 Dec 11.00 commented
  // 15 Dec 00.00 Tie interior surface temps to zone air
for i in 1:3 loop
  HeatedZone.Boundaries[i].TBoundIn = HeatedZone.TZone;
end for;
*/

  // Water loop
  connect(supply.ports[1], hydSys.port_a)
    annotation (Line(points={{-80,0},{-37.6,0}},
                                               color={0,127,255}));

  connect(hydSys.port_b, ret.ports[1])
    annotation (Line(points={{-2,0},{6,0},{6,-72},{10,-72}},
                                                         color={0,127,255}));

  // Thermal connection to zone
  // 16 Dec 13.00 connect(hydSys.port_zone, HeatedZone.portInt)
  connect(TZoneRefConst.y, heatedZone.TZoneRef) annotation (Line(points={{75,-12},
          {80,-12},{80,31.4},{68,31.4}}, color={0,0,127}));
  connect(WindowShadingConst[1:3].y, heatedZone.WindowShading[1:3]) annotation
    (Line(points={{37,-12},{46,-12},{46,14},{48,14},{48,18.7}}, color={0,0,127}));
  connect(hydSys.port_zone, heatedZone.heatPort) annotation (Line(points={{-20,18},
    {-20,32.2},{47.4,32.2}}, color={191,0,0}));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<h4>Test Hydronic Zone</h4>
<p><b>Test File for Hydronics</b></p>
<li>
Oct 17, 2025, by Karthik Murugesan
</li>
<p><b>Added components for testing hydronics</b></p>
</html>"));
end TestHydronicZone;
