package Examples     
  extends .Modelica.Icons.Package;
  annotation(uses(Modelica(version = "4.0.0"),VaporCycle(version = "2.12")),
      Documentation(revisions="<html><!--COPYRIGHT--></html>",info = "<html>
  <p><strong><span style=\"font-size: 18px;\">Aerospace content</span></strong></p><p><a href=\"https://modelon.com/modelon-impact/\">Modelon Impact</a> offers a wide range of physical modeling libraries addressing <a href=\"https://modelon.com/industries/aerospace-systems-modeling-and-simulation-software/\">aerospace industry needs</a>. This package includes an introductory collection of industry vertical examples and workflows. The package is divided into the following sub-packages.</p>
  
  <p><strong>Hydrogen Powered Aircraft</strong></p>
  <p>This vertical covers the cryogenic and gaseous hydrogen storage with its distribution and regulation <a href=\"modelica://VaporCycle\"><img src=\"modelica://Modelon/Resources/Images/Icons/VCL.png\" width=\"32\" height=\"32\"></a>. This aspect also includes the Hydrogen Tank Design models and workflows (see details below). Additionally, this vertical addresses the fuel cell and its conversion of hydrogen into electrical energy <a href=\"modelica://FuelCell\"><img src=\"modelica://Modelon/Resources/Images/Icons/FCL.png\" width=\"32\" height=\"32\"></a>, liquid cooling loops using incompressible coolants such as polypropylene glycol-water mixtures (PGW) <a href=\"modelica://LiquidCooling\"><img src=\"modelica://Modelon/Resources/Images/Icons/LCL.png\" width=\"32\" height=\"32\"></a>, various detailed heat exchanger models <a href=\"modelica://HeatExchanger\"><img src=\"modelica://Modelon/Resources/Images/Icons/HXL.png\" width=\"32\" height=\"32\"></a>, the aircraft performance of hydrogen powered concepts <a href=\"modelica://AircraftDynamics\"><img src=\"modelica://Modelon/Resources/Images/Icons/ADL.png\" width=\"32\" height=\"32\"></a>, hydrogen-fired gas turbines <a href=\"modelica://JetPropulsion\"><img src=\"modelica://Modelon/Resources/Images/Icons/JPL.png\" width=\"32\" height=\"32\"></a>, and the electric power generation and distribution <a href=\"modelica://Electrification\"><img src=\"modelica://Modelon/Resources/Images/Icons/EL.png\" width=\"32\" height=\"32\"></a>.</p>
  <p><img src=\"modelica://Examples/Resources/Images/HydrogenPoweredAircraftWithFocus.png\" width=\"640\" height=\"360\">
</p>

<p><strong>Hydrogen Tank Design example models</strong></p>
  <p>To complement the core hydrogen tank design workflows explained and exercised in the tutorial, these examples offer additional and related analyses</p>
  <ul>
  <li><a href=\"modelica://Examples.TankPressurizationAndBoilOff\">Transient simulation of tank self-pressurization as heat leaks inside including opening of a boil-off valve</a></li>    
  <li><a href=\"modelica://Examples.LiquidWithdrawal_Huete_5h\">Analysis of pressure and temperature evolution inside a liquid hydrogen tank as liquid is withdrawn at different rates</a></li>
  <li><a href=\"modelica://Examples.SelfPressurization_Hasan\">Comparison between model predictions and two sets of experimental results for a liquid hydrogen tank</a></li>
  <li><a href=\"modelica://Examples.TankFillingVerticalCylinder\">Assessment of a liquid hydrogen tank filling schedule via simulation</a></li>
  <li><a href=\"modelica://Examples.HydrogenEvaporation\">Design of a heat exchanger evaporating liquid hydrogen</a></li>
  </ul>
  
  <p><strong>Hydrogen Tank Design tutorial models</strong></p>
  <p>The models built up as part of the tutorial are provided in the accompanying <a href=\"modelica://Tutorial\">hydrogen tank design tutorials package</a>. 

  
  <p><strong>Additional examples</strong></p>
  <p>Additional examples are available in <a href=\"IndustryExamples.Aerospace\">IndustryExamples</a> on <a href=\"IndustryExamples.Aerospace.ThermalAndFuel\">Thermal management and fuel</a>, <a href=\"IndustryExamples.Aerospace.ActuationAndGears\">actuation and landing gear</a>, <a href=\"IndustryExamples.Aerospace.Propulsion\">propulsion and power</a> and <a href=\"IndustryExamples.Aerospace.Aircraft\">aircraft dynamics and performance</a>. 
</p>
  
<p>Additional aerospace content is available in the <a href=\"http://help.modelon.com/latest/verticals/use_cases_tutorials/\">tutorials</a> of Modelon Help Center.</p>
</html>")); 
end Examples;