TODO:
* Autonomous
  * port over auto code from previous python code
  * pathplanner (https://pathplanner.dev/home.html)
    * move the robot config to the constants file

* reef managment subsystem
  * Coral intake 
    * simple motor control: done
    * optical sensors
      * once they detect the piece of coral, have the coral intake prime it for deployment, detect it has reached the end via a second set of opticals sensors (see if possible without a second set)
  * algae removal
    * simple motor control
    * bumper detection + roll up (when the jaw runs into the reef)

* photon vision
  * April Tag detection
  * pose estimation
  * reef alignment

* elevator
  * Program actual set points
  * replace braking with active position alignment
    * create a variable with setgoalPosition, either use a bang bang controller or PID controller, or maybe just use the goto method we have

* drive
  * quality of life: fix the bug that has the wheels snap to position when going to deadband
    * look into advantage scope

* keybinds
  * possible optimizations of the keybinds via the CommandXboxController.
  * add a secondary controller, or a way to implemnet it

