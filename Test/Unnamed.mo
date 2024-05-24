within Test;
model Unnamed
    .Modelica.Mechanics.Translational.Components.SpringDamper springDamper(c = 10,d = 5,s_rel0 = 0.1) annotation(Placement(transformation(extent = {{-10.0,-10.0},{10.0,10.0}},origin = {0.0,0.0},rotation = -90.0)));
    .Modelica.Mechanics.Translational.Components.Fixed fixed annotation(Placement(transformation(extent = {{-10.0,-10.0},{10.0,10.0}},origin = {-0.2222222222222321,37.77777777777778},rotation = -180.0)));
    .Modelica.Mechanics.Translational.Components.Mass mass(m = 5) annotation(Placement(transformation(extent = {{-10.0,-10.0},{10.0,10.0}},origin = {-0.22222222222221077,-56.22222222222223},rotation = -90.0)));
equation
    connect(fixed.flange,springDamper.flange_a) annotation(Line(points = {{-0.2222222222222321,37.77777777777803},{-0.2222222222222321,10},{-2.220446049250313e-15,10}},color = {0,127,0}));
    connect(springDamper.flange_b,mass.flange_a) annotation(Line(points = {{2.220446049250313e-15,-10},{-0.22222222222221344,-10},{-0.22222222222221344,-46.22222222222213}},color = {0,127,0}));
    annotation(Icon(coordinateSystem(preserveAspectRatio = false,extent = {{-100.0,-100.0},{100.0,100.0}}),graphics = {Rectangle(lineColor={0,0,0},fillColor={230,230,230},fillPattern=FillPattern.Solid,extent={{-100.0,-100.0},{100.0,100.0}}),Text(lineColor={0,0,255},extent={{-150,150},{150,110}},textString="%name")}));
end Unnamed;
