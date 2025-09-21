// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import frc.robot.Constants.ReefConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ReefSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_coral;
  SparkMax m_elevator_follower;

  public ReefSubsystem(){
    m_coral = new SparkMax(ReefConstants.kCoralCanId, MotorType.kBrushless);
    
  }
  
  /**
   * Method to lift the elevator using joystick info.
   *
   */
  public void moveCoral(double xSpeed) {
    m_coral.set(xSpeed);
  }

  public void moveCoral_stop() {
    m_coral.set(0.0);
  }

  @Override
  public void periodic(){
    
  }
}
