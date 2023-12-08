within ExpandableConnectors.Components;
model Plant
  extends Modelica.Electrical.Machines.Icons.Machine;
  Modelica.Electrical.Analog.Basic.Resistor resistor annotation (Placement(transformation(extent={{-50,20},{-30,40}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor annotation (Placement(transformation(extent={{-20,20},{0,40}})));
  Modelica.Electrical.Analog.Basic.RotationalEMF emf annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia
    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(transformation(extent={{-80,-70},{-60,-50}})));
  Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
    annotation (Placement(transformation(extent={{-20,-40},{-40,-20}})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-70,0})));
    .Modelica.Blocks.Interfaces.RealInput voltage annotation(Placement(transformation(extent = {{-140.01877934272304,-20.103286384976506},{-100.01877934272304,19.896713615023494}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Blocks.Interfaces.RealOutput speed annotation(Placement(transformation(extent = {{99.89671361502351,-9.690140845070434},{119.89671361502351,10.309859154929566}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Blocks.Interfaces.RealOutput current annotation(Placement(transformation(extent = {{100.26291079812214,-69.64319248826294},{120.26291079812214,-49.64319248826295}},origin = {0.0,0.0},rotation = 0.0)));
equation
  connect(inertia.flange_a, emf.flange) annotation (Line(points={{30,0},{20,0}}, color={0,0,0}));
  connect(emf.p, inductor.n) annotation (Line(points={{10,10},{10,30},{0,30}}, color={0,0,255}));
  connect(inductor.p, resistor.n) annotation (Line(points={{-20,30},{-30,30}}, color={0,0,255}));
  connect(emf.n, currentSensor.p) annotation (Line(points={{10,-10},{10,-30},{-20,-30}}, color={0,0,255}));
  connect(speedSensor.flange, inertia.flange_b) annotation (Line(points={{60,0},{50,0}}, color={0,0,0}));
  connect(signalVoltage.p, resistor.p) annotation (Line(points={{-70,10},{-70,30},{-50,30}}, color={0,0,255}));
  connect(signalVoltage.n, currentSensor.n) annotation (Line(points={{-70,-10},{-70,-30},{-40,-30}}, color={0,0,255}));
  connect(ground.p, signalVoltage.n) annotation (Line(points={{-70,-50},{-70,-10}}, color={0,0,255}));
    connect(signalVoltage.v,voltage) annotation(Line(points = {{-82,2.6645352591003757e-15},{-101.00938967136152,2.6645352591003757e-15},{-101.00938967136152,-0.10328638497650822},{-120.01877934272304,-0.10328638497650822}},color = {0,0,127}));
    connect(speedSensor.w,speed) annotation(Line(points = {{81,0},{105.57042253521128,0},{105.57042253521128,0.3098591549295655},{109.89671361502351,0.3098591549295655}},color = {0,0,127}));
    connect(current,currentSensor.i) annotation(Line(points = {{110.26291079812214,-59.64319248826294},{-30,-59.64319248826294},{-30,-41}},color = {0,0,127}));
end Plant;
