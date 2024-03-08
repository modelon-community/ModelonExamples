package AircraftThermalManagement     extends .Modelica.Icons.Package;
    annotation (uses(Modelica(version = "4.0.0"),VaporCycle(version = "2.12"),EnvironmentalControl(version = "3.16"),Modelon(version = "4.4"),LiquidCooling(version = "2.12")),
    Documentation(revisions="<html><!--COPYRIGHT--></html>",info = "<html>
  <p><strong><span style=\"font-size: 18px;\">Aircraft thermal management</span></strong></p><p>Plan and assess cooling systems ranging from light air cycles (“environmental control systems”) to highly efficient vapor cycles. Analyze their dynamics and performance as standalone systems or integrated with liquid cooling systems. Consider different heat exchangers, compressors, turbines, pumps, and ejectors. Select suitable refrigerants and coolants.
  </p>
  <p>
  The following examples are available here. Open the relevant libraries for additional ones by clicking the library icon below.
  </p>  
  
  <p><strong>Vapor Cycle and Heat Pump</strong> <a href=\"modelica://VaporCycle\"><img src=\"modelica://Modelon/Resources/Images/Icons/VCL.png\" width=\"32\" height=\"32\"></a></p>
  <p>An example of a vapor cycle for refrigeration or heat pump is provided with
  </p><ul>
  <li><a href=\"modelica://AircraftThermalManagement.EvaporatorSizing\">Vapor cycle sizing by computing required heat transfer area</a></li>
  <li><a href=\"modelica://AircraftThermalManagement.AirConditioningSBTL\">Transient performance analysis of superheating in a vapor compression cycle</a></li>
  </ul>
  <p><br></p>

  
  <p><strong>Air Cycle and Environmental Control System including cabin and structural compartments </strong> <a href=\"modelica://EnvironmentalControl\"><img src=\"modelica://Modelon/Resources/Images/Icons/ECL.png\" width=\"32\" height=\"32\"></a></p>
  <p>Examples of air cycle refrigeration and environmental control are provided with
  </p><ul>
  <li><a href=\"modelica://AircraftThermalManagement.AirGenerationPack\">Air generation pack dynamic simulation</a></li>
  <li><a href=\"modelica://AircraftThermalManagement.CommercialCabin\">Computation of thermal loads for a commercial aircraft cabin</a></li>
  </ul>
  <p><br></p>
  
  <p><strong>Liquid Cooling Cycle</strong> <a href=\"modelica://LiquidCooling\"><img src=\"modelica://Modelon/Resources/Images/Icons/LCL.png\" width=\"32\" height=\"32\"></a></p>
  <p>Examples of liquid cooling cycles using coolants such as polypropylene glycol-water mixtures (PGW) or polyalphaolefin (PAO) are provided with
  </p><ul>
  <li><a href=\"modelica://AircraftThermalManagement.OrificeSizing\">Circuit design and restriction sizing</a><br></li>
  </ul>
  <p><br></p>
 
</html>"),Icon(graphics = {Rectangle(lineColor={200,200,200},fillColor={248,248,248},fillPattern=FillPattern.HorizontalCylinder,extent={{-100.0,-100.0},{100.0,100.0}},radius=25.0),Rectangle(lineColor={128,128,128},extent={{-100.0,-100.0},{100.0,100.0}},radius=25.0),Bitmap(fileName="modelica://AircraftThermalManagement/Resources/Images/Icons/Thermal_Fuel_vertical.png",origin={0,-4},extent={{-68,-67},{68,67}})}));
end AircraftThermalManagement;