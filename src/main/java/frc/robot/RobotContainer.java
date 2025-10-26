// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.ADXL345_I2C.Axes;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ReefSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems
  private final SendableChooser<Command> autoChooser;
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ReefSubsystem m_reef = new ReefSubsystem();
  

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_commanddriverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  //simulation
                                                                                                          
  //photon cam
  // java -jar "C:\FRC Code\PhotonVision\photonvision-v2025.3.1-winx64.jar"    
  PhotonCamera frontCamera;
  PhotonCamera rearCamera;
  
  // differnet speed modes
  double speedScaleHigh = 1.0;
  double speedScaleLow= 0.33;
  double speedScale = speedScaleHigh;
  SequentialCommandGroup deployCoralCommmand= new RunCommand(
        () -> m_reef.moveCoral(0.165),
        m_elevator).withTimeout(1.0).andThen(new InstantCommand(
          () -> m_reef.moveCoral_stop(),m_elevator));
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //CameraServer.startAutomaticCapture();
    frontCamera = new PhotonCamera("frontCam");
    rearCamera = new PhotonCamera("rearCam");
    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      // fieldRelative
      new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY()*speedScale, OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX()*speedScale, OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX()*speedScale, OIConstants.kDriveDeadband),
            true),
        m_robotDrive));

    m_reef.setDefaultCommand(
        new RunCommand(
            () -> m_reef.autoLoadCoral(),
            m_reef)
    );
    /*
    m_elevator.setDefaultCommand(
        new RunCommand(
            () -> m_elevator.stayAtPosition(),
            m_elevator)
    );
    */
    NamedCommands.registerCommand("LiftElevatorCoral4", new RunCommand(
        () -> m_elevator.goToPosition(-35),//is inverted because upwards is negative
        m_elevator));
        autoChooser = AutoBuilder.buildAutoChooser();
    
    NamedCommands.registerCommand("DeployCoral", deployCoralCommmand);
    

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  
  public void periodic() {
    System.out.println("test");
    var frontCamResult = frontCamera.getLatestResult();
    int frontTargetId = 0;
    if(frontCamResult.hasTargets()){
      List<PhotonTrackedTarget> targets = frontCamResult.getTargets();
      PhotonTrackedTarget frontTarget = frontCamResult.getBestTarget();
      frontTargetId = frontTarget.getFiducialId();
      System.out.println(frontTargetId);
    }
    SmartDashboard.putNumber("Front target id",frontTargetId);

    var rearCamResult = rearCamera.getLatestResult();
    int rearTargetId = 0;
    if(rearCamResult.hasTargets()){
      List<PhotonTrackedTarget> targets = rearCamResult.getTargets();
      PhotonTrackedTarget rearTarget = rearCamResult.getBestTarget();
      rearTargetId = rearTarget.getFiducialId();
      System.out.println(rearTargetId);
    }
    SmartDashboard.putNumber("Rear target id",rearTargetId);
    
    //boolean hasTargets = result.hasTargets();
    // java -jar "C:\FRC Code\PhotonVision\photonvision-v2025.3.1-winx64.jar"
    
    /* 
    for (var target : result.getTargets()) {
      targetId = target.getFiducialId();
      
      if (target.getFiducialId() == 22) {
        
          // Found Tag 7, record its information
          double targetYaw = target.getYaw();
          double targetRange =
          PhotonUtils.calculateDistanceToTargetMeters(
            0.5, // Measured with a tape measure, or in CAD.
            1.435, // From 2024 game manual for ID 7
            Math.toRadians(-30.0), // Measured with a protractor, or in CAD.
            Math.toRadians(target.getPitch()));

          //targetVisible = true;
      }
          
    }
    */
  }

  public static Trigger triggerButton (XboxController controller, XboxController.Axis axis) {
    return new Trigger(() -> controller.getRawAxis(axis.value) >= Constants.OIConstants.kDriveDeadband);
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
   * and then calling passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Button.kStart.value)
    .onTrue(new RunCommand(
      () -> m_reef.moveCoral(0.165),
      m_elevator).withTimeout(1.0).andThen(new InstantCommand(
        () -> m_reef.moveCoral_stop(),
        m_elevator)));

    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.driveResetEncoders(),
            m_robotDrive)).whileTrue(new RunCommand(
                () -> m_elevator.elevatorResetEncoders(),m_elevator));
    //m_commanddriverController.rightTrigger().whileTrue

    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));         

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
            () -> m_reef.moveCoral(-0.165),m_reef))// amke sure left bumper pulls coral in, maybe have it slower
            .onFalse(new InstantCommand(
                () -> m_reef.moveCoral_stop(),m_reef));

    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_reef.moveCoral(0.165),m_reef))
            .onFalse(new InstantCommand(
                () -> m_reef.moveCoral_stop(),m_reef));

    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new RunCommand(
            () -> m_elevator.goToPosition(-35),//is inverted because upwards is negative
            m_elevator)).onFalse(new InstantCommand(
                () -> m_elevator.lift_stop(),
                m_elevator).andThen(new InstantCommand(()-> m_elevator.setGoalPosition(),m_elevator)));

    new JoystickButton(m_driverController, Button.kLeftStick.value)
        .onTrue(new InstantCommand(
            () -> changeScale())); //may cause errors because non static

    triggerButton(m_driverController,Axis.kLeftTrigger).whileTrue(new RunCommand(
        () -> m_elevator.lift(m_driverController.getRawAxis(Axis.kLeftTrigger.value)),
        m_elevator)).onFalse(new InstantCommand(
            () -> m_elevator.lift_stop(),
            m_elevator).andThen(new InstantCommand(()-> m_elevator.setGoalPosition(),m_elevator)));

    triggerButton(m_driverController,Axis.kRightTrigger).whileTrue(new RunCommand(
        () -> m_elevator.lift(-m_driverController.getRawAxis(Axis.kRightTrigger.value)),
        m_elevator)).onFalse(new InstantCommand(
            () -> m_elevator.lift_stop(),
            m_elevator).andThen(new InstantCommand(()-> m_elevator.setGoalPosition(),m_elevator)));
    
  }

  public void changeScale(){
    if(speedScale == speedScaleHigh){
      speedScale = speedScaleLow;
    } else {
      speedScale = speedScaleHigh;
    }
  }
}
