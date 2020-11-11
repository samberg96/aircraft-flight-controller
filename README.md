# aircraft-flight-controller
Simulink/Matlab model of linear and nonlinear flight controller for Boeing 747.

This project was completed as part of a University course on aircraft dynamics and control. The goal was to design a flight controller (consisting of an altitude controller and heading controller) capable of the maneuvering the aircraft in a smooth and stable manner. A linearized Simulink was developed to tune the controller, and a nonlinear simulation was developed to further evaluate the performance. The full report writeup is available for further readings.

The linearized aircraft dynamics are decomposed into longitudinal and lateral components, allowing us to independently design the heading and altitude controllers. Excepts from the Simulink models are shown below.

## Altitude Controller

