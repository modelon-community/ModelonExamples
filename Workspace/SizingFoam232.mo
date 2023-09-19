within Workspace;
model SizingFoam232
    .VaporCycle.Experiments.CryogenicH2.TankSizing.Tanks.DormantFoam dormantFoam(redeclare replaceable record Material2 = .VaporCycle.Experiments.CryogenicH2.TankSizing.Tanks.Layers.MaterialProperties.PolyurethaneFoam,sf = 2.2,p_max = 404e3,V_i = 0.13,p_filling = 165.5e3,sizeAndShape = VaporCycle.Experiments.CryogenicH2.TankSizing.Tanks.Utilities.SizeAndShape.MassAndRadius,massHydAvail_prscr = 13620,r_outer_par = 4.67 / (2),insulationSizing = VaporCycle.Experiments.CryogenicH2.TankSizing.Tanks.Utilities.InsulationSizing.Dormancy) annotation(Placement(transformation(extent = {{-52.0,2.0},{-32.0,22.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T = 288.15) annotation(Placement(transformation(extent = {{-104,2},{-84,22}},origin = {0,0},rotation = 0)));
equation
    connect(fixedTemperature.port,dormantFoam.port_amb) annotation(Line(points = {{-84,12},{-50,12}},color = {191,0,0}));
    annotation(Icon(coordinateSystem(preserveAspectRatio = false,extent = {{-100.0,-100.0},{100.0,100.0}}),graphics = {Rectangle(lineColor={0,0,0},fillColor={230,230,230},fillPattern=FillPattern.Solid,extent={{-100.0,-100.0},{100.0,100.0}}),Text(lineColor={0,0,255},extent={{-150,150},{150,110}},textString="%name")}));
end SizingFoam232;
