// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  TalonFX m_leftMotor = new TalonFX(Constants.Drive.k_leftDriveMotor);
  TalonFX m_leftFollower = new TalonFX(Constants.Drive.k_leftDriveMotorFollower);
  TalonFX m_rightMotor = new TalonFX(Constants.Drive.k_rightDriveMotor);
  TalonFX m_rightFollower = new TalonFX(Constants.Drive.k_rightDriveMotorFollower);
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftMotor.configFactoryDefault();
    m_leftFollower.configFactoryDefault();
    m_rightMotor.configFactoryDefault();
    m_rightFollower.configFactoryDefault();

    m_leftFollower.follow(m_leftMotor);
    m_rightFollower.follow(m_rightMotor);

    m_leftMotor.setInverted(true);
    m_leftFollower.setInverted(true);
    m_rightMotor.setInverted(false);
    m_rightFollower.setInverted(false);
  }

  public void setPower(double leftPower, double rightPower) {
    m_leftMotor.set(ControlMode.PercentOutput, leftPower);
    m_rightMotor.set(ControlMode.PercentOutput, rightPower);
  }

  public void setBrakeMode(boolean brake) {
    NeutralMode mode = brake? NeutralMode.Brake : NeutralMode.Coast;
    
    m_leftMotor.setNeutralMode(mode);
    m_leftFollower.setNeutralMode(mode);
    m_rightMotor.setNeutralMode(mode);
    m_rightFollower.setNeutralMode(mode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
