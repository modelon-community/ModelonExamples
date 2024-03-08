within AircraftThermalManagement;
model AirConditioningSBTL "Transient performance analysis of superheating in a vapor compression cycle"
    extends .VaporCycle.Experiments.AirConditioningSBTL;
annotation (Documentation(info="<html>
<h1>Transient performance analysis of superheating</h1>
<p>This example model shows the superheating and other dynamic results.</p>
<p><img src=\"modelica://AircraftThermalManagement/Resources/Images/Subsystems/VaporCycleDynamic.png\" alt=\"modelica://AircraftThermalManagement/Resources/Images/Subsystems/VaporCycleDynamic.png\"></p>
<p>Click on the following link to open this example documentation: <a href=\"modelica://VaporCycle.Experiments.AirConditioningSBTL\">VaporCycle.Experiments.AirConditioningSBTL</a></p> </html>", revisions="<html><!--COPYRIGHT--></html>"),
     experiment(
      StopTime=800,
      __Dymola_NumberOfIntervals=5000,
      Tolerance=1e-07));
end AirConditioningSBTL;
