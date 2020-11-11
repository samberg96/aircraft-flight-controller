# aircraft-flight-controller
Simulink/Matlab model of linear and nonlinear flight controller for Boeing 747.

This project was completed as part of a University course on aircraft dynamics and control. The goal was to design a flight controller (consisting of an altitude controller and heading controller) capable of the maneuvering the aircraft in a smooth and stable manner. A linearized Simulink was developed to tune the controller, and a nonlinear simulation was developed to further evaluate the performance. The full report writeup is available for further readings.

- Phase1.pdf (linear)
- Phase2.pdf (nonlinear)

The linearized aircraft dynamics are decomposed into longitudinal and lateral components, allowing us to independently design the heading and altitude controllers. Four inputs are used to control the aircraft (throttle, rudder, aileron, elevator). In the linearized model, the throttle/elevator combo controls the longitudinal system (i.e. altitude control) and the rudder/aileron combo controls the lateral system (i.e. heading control). In this example, the aircraft is held fixed at Mach 0.8.

Excepts from the Simulink models are shown below.

## Altitude Controller
![Altitude Controller](/img/altitude_controller.JPG)
Except from the Linear_Model.slx Simulink file. The linearized longitudinal dynamics are used to design the altitude controller.


## Heading Controller
![Heading Controller](/img/heading_controller.JPG)
Except from the Linear_Model.slx Simulink file. The linearized lateral dynamics are used to design the heading controller.


## Nonlinear Model
![Nonlinear Model](/img/nonlinear_sim.JPG)
Excerpt from the Nonlinear_Model.slx Simulink file. In the nonlinear simulation, the linearized dynamics are replaced by the coupled nonlinear dynamics.

