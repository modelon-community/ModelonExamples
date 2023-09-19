within Workspace.Experiment;

model FlightRangeBase
    extends .Workspace.Experiment.TankTestBase(storageTank(initFromLevel = false,rel_level_start = 0.01,enable_liquidHeatPort = true));
    .Modelon.ThermoFluid.Sources.Environment_Q environment_Q(paraOption_Qflow = true) annotation(Placement(transformation(extent = {{36.0,-16.0},{16.0,4.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Blocks.Sources.Ramp ramp(height = 1,startTime = 0) annotation(Placement(transformation(extent = {{78.0,-16.0},{58.0,4.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Blocks.Sources.RealExpression timeSignal(y = .Modelica.Units.Conversions.to_hour(time)) annotation(Placement(transformation(extent = {{-178.0,-4.0},{-158.0,16.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Blocks.Math.Gain gain(k = -1) annotation(Placement(transformation(extent = {{-99.01675869902485,-29.016758699024845},{-80.98324130097515,-10.983241300975155}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Blocks.Tables.CombiTable1Ds altitudeTable(tableOnFile = true,tableName = "Altitude",extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint) annotation(Placement(transformation(extent = {{-142,38},{-122,58}},origin = {0,0},rotation = 0)));
    .Modelica.Blocks.Tables.CombiTable1Ds liquidFlowRateTable(tableOnFile = true,tableName = "FuelFlow",extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint) annotation(Placement(transformation(extent = {{-140.0,-20.0},{-120.0,0.0}},origin = {0.0,0.0},rotation = 0.0)));
equation
    connect(environment_Q.port[1],storageTank.heat_liquid) annotation(Line(points = {{26,4},{26,12.899999999999999},{-20.1,12.899999999999999}},color = {191,0,0}));
    connect(ramp.y,environment_Q.Qflow_in) annotation(Line(points = {{57,-6},{36,-6}},color = {0,0,127}));
    connect(gain.y,massFlowBoundary.m_flow_in) annotation(Line(points = {{-80.08156543107266,-20},{-55.60876435056908,-20},{-55.60876435056908,-21},{-43.2,-21}},color = {0,0,127}));
    connect(timeSignal.y,altitudeTable.u) annotation(Line(points = {{-157,6},{-150.5,6},{-150.5,48},{-144,48}},color = {0,0,127}));
    connect(altitudeTable.y[1],airData.alt_in) annotation(Line(points = {{-121,48},{-115,48},{-115,67.6},{-104.6,67.6}},color = {0,0,127}));
    connect(liquidFlowRateTable.u,timeSignal.y) annotation(Line(points = {{-142,-10},{-150,-10},{-150,6},{-157,6}},color = {0,0,127}));
    connect(liquidFlowRateTable.y[1],gain.u) annotation(Line(points = {{-119,-10},{-109.91005521941491,-10},{-109.91005521941491,-20},{-100.82011043882981,-20}},color = {0,0,127}));
end FlightRangeBase;
