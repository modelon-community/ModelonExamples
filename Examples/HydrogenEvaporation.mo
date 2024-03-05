within Examples;
model HydrogenEvaporation
    extends .VaporCycle.Experiments.HydrogenStorage.HydrogenEvaporation;
    annotation(Documentation(info="<html>
<h1>Transient performance analysis of evaporating heat exchanger</h1>
<p>Example heat exchanger model where liquid cryogenic hydrogen is evaporated by feeding it through a heat exchanger with ambient air as secondary fluid.</p>
<p>Click on the following link to open this example documentation: <a href=\"modelica://VaporCycle.Experiments.HydrogenStorage.HydrogenEvaporation\">VaporCycle.Experiments.HydrogenStorage.HydrogenEvaporation</a></p> </html>", revisions="<html><!--COPYRIGHT--></html>"),
     experiment(
      StopTime=1,
      Tolerance=1e-05,
      __Dymola_Algorithm="Dassl"));
end HydrogenEvaporation;
