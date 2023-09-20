within Tutorial;
model SizingMLI232
    .VaporCycle.Experiments.CryogenicH2.TankSizing.Tanks.DormantVacuum dormantVacuum(sf = 2.2,p_max = 210000,V_i = 0.13,p_filling = 165500,sizeAndShape = VaporCycle.Experiments.CryogenicH2.TankSizing.Tanks.Utilities.SizeAndShape.MassAndRadius,massHydAvail_prscr = 10220,r_outer_par = 4.67 / (2),thickness_par = 0.127) annotation(Placement(transformation(extent = {{-38.0,14.0},{-18.0,34.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T = 288.15) annotation(Placement(transformation(extent = {{-84.0,14.0},{-64.0,34.0}},origin = {0.0,0.0},rotation = 0.0)));
equation
    connect(fixedTemperature.port,dormantVacuum.port_amb) annotation(Line(points = {{-64,24},{-36,24}},color = {191,0,0}));
    annotation(Icon(coordinateSystem(preserveAspectRatio = false,extent = {{-100.0,-100.0},{100.0,100.0}}),graphics = {Rectangle(lineColor={0,0,0},fillColor={230,230,230},fillPattern=FillPattern.Solid,extent={{-100.0,-100.0},{100.0,100.0}}),Text(lineColor={0,0,255},extent={{-150,150},{150,110}},textString="%name")}));
end SizingMLI232;
