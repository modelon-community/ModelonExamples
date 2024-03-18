within Examples;
model TankFillingVerticalCylinder
    extends .VaporCycle.Experiments.HydrogenStorage.TankFillingVerticalCylinder;
    annotation(Documentation(info="<html>
<h1>Assessment of liquid hydrogen tank filling schedule</h1>
<p>In this model, a tank filling schedule is provided. It's performance in terms of filling time and amount of evaporated hydrogen is then assessed via simulation.</p>
<p>Click on the following link to open this example documentation: <a href=\"modelica://VaporCycle.Experiments.HydrogenStorage.TankFillingVerticalCylinder\">VaporCycle.Experiments.HydrogenStorage.TankFillingVerticalCylinder</a></p> </html>", revisions="<html><!--COPYRIGHT--></html>"),
     experiment(
      StopTime=6000,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"));      
end TankFillingVerticalCylinder;
