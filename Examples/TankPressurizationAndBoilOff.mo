within Examples;
model TankPressurizationAndBoilOff
    extends .VaporCycle.Experiments.CryogenicH2.TankPressurizationAndBoilOff;
    annotation(Documentation(info="<html>
<h1>Transient simulation of tank self-pressurization</h1>
<p>Boundary conditions are set up to represent heat leaking into the tank. Additionally, a boil-off valve is included, which allows venting the tank as maximum pressure is reached. Together with an inhomogeneous tank, a transient simulation of tank self-pressurization is achieved.</p>
<p>Click on the following link to open this example documentation: <a href=\"modelica://VaporCycle.Experiments.CryogenicH2.TankPressurizationAndBoilOff\">VaporCycle.Experiments.CryogenicH2.TankPressurizationAndBoilOff</a></p> </html>", revisions="<html><!--COPYRIGHT--></html>"),
     experiment(StopTime=86400, __Dymola_Algorithm="Dassl"));      
end TankPressurizationAndBoilOff;
