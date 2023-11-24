within Examples;
model SelfPressurization_Hasan
    extends .VaporCycle.Experiments.CryogenicH2.SelfPressurization_Hasan;
    annotation(Documentation(info="<html>
<h1>Comparison of model predictions and measurements</h1>
<p>In this model, a comparison between two sets of experimental results for a liquid hydrogen tank and model predictions is provided.</p>
<p>Click on the following link to open this example documentation: <a href=\"modelica://VaporCycle.Experiments.CryogenicH2.SelfPressurization_Hasan\">VaporCycle.Experiments.CryogenicH2.SelfPressurization_Hasan</a></p> </html>", revisions="<html><!--COPYRIGHT--></html>"),
     experiment(
      StopTime=50400,
      Tolerance=1e-05,
      __Dymola_Algorithm="Dassl"));      
end SelfPressurization_Hasan;
