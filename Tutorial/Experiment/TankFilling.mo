within Tutorial.Experiment;
model TankFilling
    extends .Tutorial.Experiment.TankTestBase(storageTank(initFromLevel = true,rel_level_start = 0.01),checkValve(dp_open = p_max - 101305));
    .Modelica.Blocks.Sources.RealExpression altitude(y = 0) annotation(Placement(transformation(extent = {{-146.0,20.0},{-126.0,40.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Blocks.Sources.TimeTable timeTable(table = [0,0;t_fill_start,0;t_fill_start,fill_rate;t_fill_start + t_fill,fill_rate;t_fill_start + t_fill,0]) annotation(Placement(transformation(extent = {{-126.0,-32.0},{-106.0,-12.0}},origin = {0.0,0.0},rotation = 0.0)));
    parameter .Modelica.Units.SI.Mass M_liq_fill "The amount of fuel to transfer into storage tank" annotation(Dialog(group = "Filling experiment"));
    parameter .Modelica.Units.SI.Time t_fill_start "The time at which to start filing the storage tank" annotation(Dialog(group = "Filling experiment"));
    parameter .Modelica.Units.SI.MassFlowRate fill_rate "The mass flow rate of filling" annotation(Dialog(group = "Filling experiment"));
    parameter .Modelica.Units.SI.Time t_fill = M_liq_fill / fill_rate "The time is takes to fill the tank with M_liq_fill" annotation(Dialog(group = "Filling experiment"));
equation
    connect(massFlowBoundary.m_flow_in,timeTable.y) annotation(Line(points = {{-43.2,-21},{-105,-21},{-105,-22}},color = {0,0,127}));
    connect(airData.alt_in,altitude.y) annotation(Line(points = {{-104.6,67.6},{-109,67.6},{-109,30},{-125,30}},color = {0,0,127}));
end TankFilling;
