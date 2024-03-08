within AircraftThermalManagement;
model OrificeSizing "Circuit design and restriction sizing"
  extends .LiquidCooling.Experiments.FlowBalancing.OrificeSizing;
  annotation (
      experiment(StopTime=40, __Dymola_Algorithm="Cvode"),
      Documentation(info="<html>
<h1>Circuit design and restriction sizing</h1>
<p>This example demonstrates the sizing of flow calibration orifices in a cooling circuit with several branches and a single supply.</p>
<p><img src=\"modelica://AircraftThermalManagement/Resources/Images/Subsystems/FlowCalibration.png\" alt=\"modelica://AircraftThermalManagement/Resources/Images/Subsystems/FlowCalibration.png\"></p>
<h2>Notice</h2>
<p>This is an experiment that can be run in steady-state and dynamic simulation mode. By default, it is not intended
to be used in dynamic simulations and is instead meant to be simulated with the 
<a href=\"modelica://LiquidCooling.Information.UsersGuide.GeneralInstructions.PbS\">Physics-based Solving</a>
supported Steady-State execution in Modelon Impact.</p>
<p>Please refer to the <a href=\"modelica://LiquidCooling.Experiments.FlowBalancing.OrificeSizing\">documentation</a> for more details.</p>
<p>Click on the following link to open this example documentation: <a href=\"modelica://LiquidCooling.Experiments.FlowBalancing.OrificeSizing\">LiquidCooling.Experiments.FlowBalancing.OrificeSizing</a></p>
</html>", revisions="<html><!--COPYRIGHT--></html>"));    
end OrificeSizing;
