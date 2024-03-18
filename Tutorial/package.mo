package Tutorial
  extends Modelica.Icons.Package;
  annotation(uses(Modelica(version = "4.0.0"),VaporCycle(version = "2.12"),Modelon(version = "4.4"),ThermalPower(version = "1.26")),
      Documentation(revisions="<html><!--COPYRIGHT--></html>",info = "<html>
  <p><strong><span style=\"font-size: 18px;\">Aerospace content</span></strong></p><p><span><a href=\"https://modelon.com/modelon-impact/\">Modelon Impact</a>&nbsp;offers a wide range of physical modeling libraries addressing <a href=\"https://modelon.com/industries/aerospace-systems-modeling-and-simulation-software/\">aerospace industry needs</a>. This package contains the models built as part of the Hydrogen Tank Design tutorials, which can serve as reference solutions and intermediate check points. It contains the following models.</span></p>
  
  <p><strong>Hydrogen Tank Design tutorial models</strong></p>
  <p>The following models are built in the different steps of the hydrogen tank design tutorial</p>
  <ul>
  <li>The <a href=\"https://help.modelon.com/latest/tutorials/hydrogen_storage/tank_sizing/\">tank sizing workflow</a> leads to models representing <a href=\"modelica://Tutorial.SizingFoam232\">foam-insulated designs</a> and <a href=\"modelica://Tutorial.SizingMLI232\">multi-layer vacuum-insulated designs</a>.</li>    
  <li>The <a href=\"https://help.modelon.com/latest/tutorials/hydrogen_storage/refueling/\">refill assessment workflow</a> leads to a <a href=\"modelica://Tutorial.Experiment.TankFilling\">refill simulation</a>.</li>
  <li>The <a href=\"https://help.modelon.com/latest/tutorials/hydrogen_storage/storage/\">tank storage prediction workflow</a> is used to create the <a href=\"modelica://Tutorial.Experiment.TankPressurization\">storage and self-pressurization simulation</a></li>
  <li>The <a href=\"https://help.modelon.com/latest/tutorials/hydrogen_storage/fuel_consumption/\">flight mission and fuel consumption workflow</a> enables both a <a href=\"modelica://Tutorial.Experiment.FlightRangeFoam232\">mission prediction for the foam-insulated design</a> and <a href=\"modelica://Tutorial.Experiment.FlightRangeMLI232\">mission prediction for the vacuum-insulated design</a>.</li>
  </ul>
  <p><br></p>
  
  <p><strong>Hydrogen Powered Aircraft</strong></p>
  <p>An overview of this vertical is provided in the accompanying <a href=\"modelica://Examples\">hydrogen tank design examples package</a>. 



  
  <p><strong>Additional examples</strong></p>
  <p>Additional examples are available in <a href=\"IndustryExamples.Aerospace\">IndustryExamples</a> on <a href=\"IndustryExamples.Aerospace.ThermalAndFuel\">Thermal management and fuel</a>, <a href=\"IndustryExamples.Aerospace.ActuationAndGears\">actuation and landing gear</a>, <a href=\"IndustryExamples.Aerospace.Propulsion\">propulsion and power</a> and <a href=\"IndustryExamples.Aerospace.Aircraft\">aircraft dynamics and performance</a>. 
</p>
  
<p>Additional aerospace content is available in the <a href=\"http://help.modelon.com/latest/verticals/use_cases_tutorials/\">tutorials</a> of Modelon Help Center.</p>
</html>"));     
end Tutorial;