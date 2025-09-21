// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_elevator;
  SparkMax m_elevator_follower;

  public ElevatorSubsystem(){
    m_elevator = new SparkMax(ElevatorConstants.kElevatorCanId, MotorType.kBrushless);
    m_elevator_follower = new SparkMax(ElevatorConstants.kElevatorFollowerCanId, MotorType.kBrushless);
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    softLimitConfig
      .forwardSoftLimit(0)//positive
      .reverseSoftLimit(-ElevatorConstants.maxElevatorHeightTest)//negative(direction we want to go)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);
    globalConfig
    .apply(softLimitConfig)
      .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
      .idleMode(ElevatorConstants.kElevatorIdleMode);
      
    
    leaderConfig
      .apply(globalConfig)
      .inverted(false);
      

    followerConfig
      .apply(globalConfig)
      .follow(m_elevator,true);
    
    m_elevator.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevator_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
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
  private double calc_speed(double position){
    double distanceToPosition = position-m_elevator.getEncoder().getPosition();
    double speed = Math.max(Math.min(0.3,distanceToPosition/10), 0.3);
    if(Math.abs(distanceToPosition)<=0.5){
      speed = 0;
    }
    return speed;
  }
  public void goToPosition(double position){
    double speed = calc_speed(position);
    if(Math.abs(m_elevator.getEncoder().getPosition())>Math.abs(position)){
      
    } else {
      speed = -speed;
    }
    m_elevator.set(speed);
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Elevator Speed", m_elevator.getEncoder().getVelocity());
    SmartDashboard.putNumber("Elevator pos", m_elevator.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator current", m_elevator.getOutputCurrent());
  }
}
