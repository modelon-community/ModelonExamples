within ExpandableConnectors.Components;
model Controller
    extends Modelon.Icons.DigitalController;
    .Modelica.Blocks.Interfaces.RealOutput voltage annotation(Placement(transformation(extent = {{100.10328638497651,-9.69014084507042},{120.10328638497651,10.30985915492958}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Blocks.Interfaces.RealInput speed_measured annotation(Placement(transformation(extent = {{-20.0,-20.0},{20.0,20.0}},origin = {-40.02454539443227,-119.9154929577465},rotation = 90.0)));
    .Modelica.Blocks.Interfaces.RealInput speed_target annotation(Placement(transformation(extent = {{-140.0657276995305,-19.94366197183098},{-100.0657276995305,20.05633802816902}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Blocks.Interfaces.RealInput current_measured annotation(Placement(transformation(extent = {{-20.0,-20.0},{20.0,20.0}},origin = {39.869523242742865,-120.55861932156319},rotation = 90.0)));
    .Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI currentController(useFF = false,initType = .Modelica.Blocks.Types.Init.InitialOutput,Ti = 0.03,k = 600,constantLimits = true) annotation(Placement(transformation(extent = {{29.465940069823723,-9.861151537077422},{49.46594006982372,10.138848462922578}},rotation = 0.0,origin = {0.0,0.0})));
    .Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI speedController(yMax = 95,constantLimits = true,Ti = 0.01,k = 10000,initType = .Modelica.Blocks.Types.Init.InitialOutput) annotation(Placement(transformation(extent = {{-49.84681927346387,-9.861151537077422},{-29.846819273463872,10.138848462922578}},rotation = 0.0,origin = {0.0,0.0})));
    .Modelica.Blocks.Math.Gain gain(k = 1.6) annotation(Placement(transformation(extent = {{-12.219640848199603,-9.79253120507423},{7.780359151800397,10.20746879492577}},origin = {0.0,0.0},rotation = 0.0)));
equation
    connect(speedController.u_m,speed_measured) annotation(Line(points = {{-45.84681927346387,-11.861151537077422},{-45.84681927346387,-60.12911581143148},{-40.02454539443227,-60.12911581143148},{-40.02454539443227,-119.9154929577465}},color = {0,0,127}));
    connect(speed_target,speedController.u) annotation(Line(points = {{-120.0657276995305,0.05633802816901934},{-85.95627348649718,0.05633802816901934},{-85.95627348649718,0.1388484629225779},{-51.84681927346387,0.1388484629225779}},color = {0,0,127}));
    connect(currentController.u_m,current_measured) annotation(Line(points = {{33.46594006982372,-11.861151537077422},{33.46594006982372,-60.08307007189429},{39.869523242742865,-60.08307007189429},{39.869523242742865,-120.55861932156319}},color = {0,0,127}));
    connect(currentController.y,voltage) annotation(Line(points = {{50.46594006982372,0.1388484629225779},{80.28461322740011,0.1388484629225779},{80.28461322740011,0.3098591549295797},{110.10328638497651,0.3098591549295797}},color = {0,0,127}));
    connect(speedController.y,gain.u) annotation(Line(points = {{-28.846819273463872,0.1388484629225779},{-21.778302675128764,0.1388484629225779},{-21.778302675128764,0.20746879492577008},{-14.219640848199603,0.20746879492577008}},color = {0,0,127}));
    connect(gain.y,currentController.u) annotation(Line(points = {{8.780359151800397,0.20746879492577008},{17.878076996515034,0.20746879492577008},{17.878076996515034,0.1388484629225779},{27.465940069823723,0.1388484629225779}},color = {0,0,127}));
end Controller;
