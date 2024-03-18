within Examples;
model LiquidWithdrawal_Huete_5h
    extends .VaporCycle.Experiments.HydrogenStorage.LiquidWithdrawal_Huete_5h;
    annotation(Documentation(info="<html>
<h1>Pressure and temperature inside tank as hydrogen is extracted</h1>
<p>This model provides a prediction of pressure and temperature evoluation as liquid or vapor hydrogen is extracted.</p>
<p>Click on the following link to open this example documentation: <a href=\"modelica://VaporCycle.Experiments.HydrogenStorage.LiquidWithdrawal_Huete_5h\">VaporCycle.Experiments.HydrogenStorage.LiquidWithdrawal_Huete_5h</a></p> </html>", revisions="<html><!--COPYRIGHT--></html>"),
     experiment(
      StopTime=18000,
      Tolerance=1e-05,
      __Dymola_Algorithm="Dassl"));    
end LiquidWithdrawal_Huete_5h;
