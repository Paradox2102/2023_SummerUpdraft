// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReachSubsystem extends SubsystemBase {
  private TalonFX m_reachMotor = new TalonFX(Constants.Reach.k_reachMotor);

  /** Creates a new ReachSubsystem. */
  public ReachSubsystem() {
    m_reachMotor.configFactoryDefault();
  }

  public void setPower(double reachPower) {
    m_reachMotor.set(ControlMode.PercentOutput, reachPower);
  }
  
  //brakes

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



  /*Goals:
   * motor
   * set power, command (out & in)
   * 
   * 
   * 
   */
}
