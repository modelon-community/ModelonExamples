within VaporCycleExamples;

model AirConditioning "R-134a AC-system with controlled air temperature"


  extends .VaporCycle.Experiments.RefrigerationSystem.AirConditioning;

  annotation (
    experiment(
      StopTime=800,
      __Dymola_NumberOfIntervals=5000,
      Tolerance=1e-007),
    __Dymola_experimentSetupOutput(equdistant=false),
    Documentation(revisions="<html>
Copyright &copy; 2004-2023, MODELON AB <br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br />This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br />or by other means have their origin from any Modelon Library.
</html>", info="<html>
<p>This is an example of an air-conditioning system experiment with R-134a as refrigerant. A vapor compression cycle with air as secondary fluid in condenser and evaporator is controlled via a variable volume displacement in the compressor to reach a pre-defined air outlet temperature at the evaporator. The superheating at the evaporator is regulated by a thermostatic expansion valve. The system is initialized with a given specific charge. During the simulation, transients are applied to the air boundary conditions at both heat exchangers. The COP (coefficient of performance) of the system is defined as the heat flow rate from the air flow at the evaporator divided by the provided compressor shaft power.This model is extended from <a href=\"modelica://VaporCycle.Experiments.RefrigerationSystem.AirConditioning\">AirConditioning</a>.</p>
<p>Plot the following variables:
<ul>
<li>Superheating at the evaporator outlet:<code class=\"modelica\">summary.superheat</code></li>
<li>Subcooling at the condenser outlet:<code class=\"modelica\">summary.subcooling</code></li>
<li>Air outlet at the evaporator (controlled variable)<code class=\"modelica\">summary.TairEvap_out</code></li>
<li>Coefficient of performance:<code class=\"modelica\">summary.COP</code></li>
<li>Suctions and discharge pressure:<code class=\"modelica\">summary.p_high</code> and <code class=\"modelica\">summary.p_low</code>
<li>Refrigerant mass flow rate:<code class=\"modelica\">summary.mflow</code></li></ul></p>
<p>Observe, that
<ul>
<li>The refrigerant charge is sufficient to ensure subcooling in all operating points.</li>
<li>The evaporator air outlet set-point is reached only under some of the boundary conditions. High cooling load (evaporator air flow and temperature) and high temperatures at the condenser reveal the systems capacity limits.</li>
<li>Increasing the air flow rate at the condenser decreases the high pressure level and increases process efficiency (COP).</li>
<li>The thermostatic expansion valve regulates the refrigerant mass flow rate to keep the superheating in a certain range but does not control it to a fixed superheating value.</li>
</ul>
</p>
</html>"));

end AirConditioning;
