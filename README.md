TODO:
*Autonomous
** pathplanner

*reef managment subsystem
** Coral intake 
***motors: done
***optical sensors: once they detect the piece of coral, have the coral intake prime it for deployment, detect it has reached the end via a second set of opticals sensors(is probably possible without a second set)
** algae removal



*photon vision:
*elevator
** Program actual set points
** replace braking with active position alignment
***create a variable with setgoalPosition, either use a bang bang controller or PID controller, or maybe just use the goto method we have



*drive
**quality of life: fix the bug that has the wheels snap to position when going to deadband

*keybinds
**possible optimizations of the keybinds via the CommandXboxController.
** add a secondary controller, or a way to implemnet it