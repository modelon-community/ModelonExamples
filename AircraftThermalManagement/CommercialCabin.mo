within AircraftThermalManagement;
model CommercialCabin "Computation of thermal loads for a commercial aircraft cabin"
    extends .Modelon.Icons.Experiment;
    extends .EnvironmentalControl.Experiments.CommercialCabin;
    annotation (experiment(
        StopTime=2000,
        Interval=4,
        __Dymola_Algorithm="Dassl",
        Tolerance=1e-05),
    Documentation(info="<html>
<h1>Aircraft Commercial Cabin</h1>
<p>This model presents an aircraft cabin with a topology typically seen on commercial or transport-type aircraft. 
On the far left, the example contains two sources of fresh and conditioned air. This air is normally provided by the 
<a href=\"modelica://EnvironmentalControl.Experiments.TwoWheelBootstrap\">refrigeration or air conditioning cycle</a>. 
Typically, this is some air cycle such as a Bootstrap Cycle (either bleed-air driven or using a motorized Air Cycle Machine).</p><p><img src=\"modelica://AircraftThermalManagement/Resources/Images/Subsystems/CommercialCabin.PNG\" alt=\"modelica://AircraftThermalManagement/Resources/Images/Subsystems/CommercialCabin.PNG\"><br></p>
<p>Click on the following link to open this example documentation: 
<a href=\"modelica://EnvironmentalControl.Experiments.CommercialCabin\">EnvironmentalControl.Experiments.CommercialCabin</a></p>
</html>", revisions="<html><!--COPYRIGHT--></html>"));
end CommercialCabin;
