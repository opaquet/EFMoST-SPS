# EFMoST-SPS
Firmware for the main control unit for the EFMoST Bioreactor.

Core is an Adruino Mega that reads all button and potentiometer states and either controls all actuators based on the given inputs (manual mode)
or conrtols all actuators based on external input over the Serial0 interface (automatic mode)

It reads all sensor values und displays the main values on the 6 displays and als writes everything out to the Serial0 interface, for a remote control 
PC to also have access to these values.

The actual automatic process control is handeled by an external PC
