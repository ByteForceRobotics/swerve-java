// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ReefConstants;


public class ReefSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  SparkMax m_coral;
  SparkMax m_algae;
  DigitalInput m_funnelBeamBreak;
  DigitalInput m_coralBeamBreak;
  boolean funnelBeamBroken;
  boolean coralBeamBroken;


  public ReefSubsystem(){
    m_coral = new SparkMax(ReefConstants.kCoralCanId, MotorType.kBrushless);
    m_algae = new SparkMax(ReefConstants.kAlgaeCanId, MotorType.kBrushless);
    m_funnelBeamBreak = new DigitalInput(ReefConstants.kFunnelBeamBreakDIO);
    m_coralBeamBreak = new DigitalInput(ReefConstants.kCoralBeamBreakDIO);
    funnelBeamBroken =  false;
    coralBeamBroken = false;
  }
  public void moveAlgae(double speed) {
    m_algae.set(speed);
  }
  public void moveAlgae_stop() {
    m_algae.set(0.0);
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
  public void passivePower(){
    m_coral.set(ReefConstants.kCoralPassivePower);
    m_algae.set(0);
  }
  public void autoLoadCoral(){
    if(funnelBeamBroken&&!coralBeamBroken){
      moveCoral(ReefConstants.kCoralAutoLoadSpeed);
      //moveAlgae(ReefConstants.kAlgaeAutoLoadSpeed);
    }
    if(coralBeamBroken){
      passivePower();
    }
  }

  @Override
  public void periodic(){
    coralBeamBroken = !m_coralBeamBreak.get();
    funnelBeamBroken  = !m_funnelBeamBreak.get();
    SmartDashboard.putBoolean("Funnel Beam Break",funnelBeamBroken);
    SmartDashboard.putBoolean("Coral Beam Break",coralBeamBroken);
  }
}
