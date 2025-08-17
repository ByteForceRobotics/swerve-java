// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_elevator;

  public ElevatorSubsystem(){
    m_elevator = new SparkMax(ElevatorConstants.kElevatorCanId, MotorType.kBrushless);
  }

  /**
   * Method to lift the elevator using joystick info.
   *
   */
  public void lift(double xSpeed) {
    m_elevator.set(xSpeed);
  }

  public void lift_stop() {
    m_elevator.set(0.0);
  }
}
