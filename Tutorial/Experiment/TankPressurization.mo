within Tutorial.Experiment;

model TankPressurization
    extends .Tutorial.Experiment.TankTestBase(storageTank(initFromLevel = false,rel_level_start = 0.01,enable_liquidHeatPort = true));
    .Modelica.Blocks.Sources.RealExpression altitude(y = 0) annotation(Placement(transformation(extent = {{-146.0,20.0},{-126.0,40.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Blocks.Sources.TimeTable timeTable(table = [0,0;1,0]) annotation(Placement(transformation(extent = {{-126.0,-32.0},{-106.0,-12.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelon.ThermoFluid.Sources.Environment_Q environment_Q(paraOption_Qflow = true) annotation(Placement(transformation(extent = {{36.0,-16.0},{16.0,4.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Blocks.Sources.Ramp ramp(height = 1e4,duration = 100,startTime = 20000) annotation(Placement(transformation(extent = {{78.0,-16.0},{58.0,4.0}},origin = {0.0,0.0},rotation = 0.0)));
equation
    connect(massFlowBoundary.m_flow_in,timeTable.y) annotation(Line(points = {{-43.2,-21},{-105,-21},{-105,-22}},color = {0,0,127}));
    connect(environment_Q.port[1],storageTank.heat_liquid) annotation(Line(points = {{26,4},{26,12.899999999999999},{-20.1,12.899999999999999}},color = {191,0,0}));
    connect(ramp.y,environment_Q.Qflow_in) annotation(Line(points = {{57,-6},{36,-6}},color = {0,0,127}));
    connect(airData.alt_in,altitude.y) annotation(Line(points = {{-104.6,67.6},{-109,67.6},{-109,30},{-125,30}},color = {0,0,127}));
end TankPressurization;
