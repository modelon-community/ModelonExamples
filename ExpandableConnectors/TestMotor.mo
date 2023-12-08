within ExpandableConnectors;
model TestMotor
    extends Modelica.Icons.Example;
    .ExpandableConnectors.Components.Plant plant annotation(Placement(transformation(extent = {{20.15962441314555,-10.103286384976542},{40.15962441314555,9.896713615023458}},origin = {0.0,0.0},rotation = 0.0)));
    .ExpandableConnectors.Components.Controller controller annotation(Placement(transformation(extent = {{-39.74647887323942,-10.103286384976542},{-19.74647887323942,9.896713615023458}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Blocks.Sources.Sine sine(f = 1) annotation(Placement(transformation(extent = {{-80.23474178403755,-9.89671361502347},{-60.23474178403755,10.10328638497653}},origin = {0,0},rotation = 0)));
equation
    connect(controller.voltage,plant.voltage) annotation(Line(points = {{-18.736150234741764,-0.07230046948358448},{-0.7023474178403646,-0.07230046948358448},{-0.7023474178403646,-0.1136150234741935},{18.157746478873243,-0.1136150234741935}},color = {0,0,127}));
    connect(sine.y,controller.speed_target) annotation(Line(points = {{-59.23474178403755,0.10328638497652953},{-41.75305164319247,0.10328638497652953},{-41.75305164319247,-0.09765258215964012}},color = {0,0,127}));
    connect(plant.speed,controller.speed_measured) annotation(Line(points = {{41.149295774647904,-0.07230046948358543},{47.35586854460099,-0.07230046948358543},{47.35586854460099,-30.69577464788736},{-33.748933412682646,-30.69577464788736},{-33.748933412682646,-12.094835680751192}},color = {0,0,127}));
    connect(plant.current,controller.current_measured) annotation(Line(points = {{41.18591549295776,-6.067605633802836},{49.912676056338036,-6.067605633802836},{49.912676056338036,-28.394829068306567},{-25.75952654896513,-28.394829068306567},{-25.75952654896513,-12.15914831713286}},color = {0,0,127}));
end TestMotor;
