        within AircraftThermalManagement;
model AirGenerationPack "Air generation pack dynamic simulation"
    extends .Modelon.Icons.Experiment;
    extends .EnvironmentalControl.Experiments.ThreeWheelBootstrap.SinglePack;
    annotation (experiment(
        StopTime=20,
        Interval=0.04,
        __Dymola_Algorithm="Dassl",
        Tolerance=1e-06), 
    Documentation(info="<html>
<h1>Aircraft Refrigeration Cycle</h1>
<p>This experiment shows a Three Wheel Boostrap Cycle air refrigeration cycle with one compressor, one turbine, and primary and main heat exchangers. It demonstrates the use of a water separator between the compressor and turbine. The benefit of having the water separator on the high-pressure end of the turbine is that it is not limited by ice build-up which results in higher cooling capacity. To ensure the high efficiency of the air cycle, an extra pair of heat exchangers (condenser and reheater) is placed before and after the water separator. The extracted water is then sprayed back into the ram air, where it is being evaporated and lowers the temperature of the inflowing cold air to the main heat exchanger.</p><p><img src=\"modelica://AircraftThermalManagement/Resources/Images/Subsystems/AirGenerationPack.PNG\" alt=\"modelica://AircraftThermalManagement/Resources/Images/Subsystems/AirGenerationPack.PNG\"><br></p>
<p>Click on the following link to open this example documentation: 
<a href=\"modelica://EnvironmentalControl.Experiments.ThreeWheelBootstrap.SinglePack\">EnvironmentalControl.Experiments.ThreeWheelBootstrap.SinglePack</a></p> </html>", revisions="<html><!--COPYRIGHT--></html>"));
end AirGenerationPack;
