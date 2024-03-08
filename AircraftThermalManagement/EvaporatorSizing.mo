within AircraftThermalManagement;
model EvaporatorSizing "Vapor cycle sizing by computing required heat transfer area"
    extends .VaporCycle.Experiments.HeatPumpSteadyState.EvaporatorSizing;
annotation (Documentation(info="<html>
<h1>Sizing by computing required evaporator heat transfer area</h1>
<p>This example demonstrates the sizing of a vapor cycle.</p>
<p><img src=\"modelica://AircraftThermalManagement/Resources/Images/Subsystems/VaporCycleSizing.png\" alt=\"modelica://AircraftThermalManagement/Resources/Images/Subsystems/VaporCycleSizing.png\"></p>
<h2>Notice</h2>
<p>This is a steady-state experiment. It is not intended
to be used in dynamic simulations and are instead meant to be simulated with the 
<a href=\"modelica://VaporCycle.Information.UsersGuide.PbS\">Physics-based Solving</a>
supported Steady-State execution in Modelon Impact.</p>
<p>Please refer to the <a href=\"modelica://VaporCycle.Experiments.HeatPumpSteadyState.Information\">documentation</a> for more details.</p>
<p>Click on the following link to open this example documentation: <a href=\"modelica://VaporCycle.Experiments.HeatPumpSteadyState.EvaporatorSizing\">VaporCycle.Experiments.HeatPumpSteadyState.EvaporatorSizing</a></p>
</html>", revisions="<html><!--COPYRIGHT--></html>"));
end EvaporatorSizing;
