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

  TalonFX m_rightDrive = new TalonFX(Constants.k_rightDrive);
  TalonFX m_rightDriveFollower = new TalonFX(Constants.k_rightDriveFollower);
  TalonFX m_leftDrive = new TalonFX(Constants.k_leftDrive);
  TalonFX m_leftDriveFollower = new TalonFX(Constants.k_leftDriveFollower);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_rightDrive.configFactoryDefault();
    m_rightDriveFollower.configFactoryDefault();
    m_leftDrive.configFactoryDefault();
    m_leftDriveFollower.configFactoryDefault();

    m_rightDriveFollower.follow(m_rightDrive);
    m_leftDriveFollower.follow(m_leftDrive);

    m_rightDrive.setInverted(false);
    m_rightDriveFollower.setInverted(false);
    m_leftDrive.setInverted(true);
    m_leftDriveFollower.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double rightPower, double leftPower) {
    m_rightDrive.set(ControlMode.PercentOutput, rightPower);
    m_leftDrive.set(ControlMode.PercentOutput, leftPower);
  }

  public void  setBrakeMode(boolean brake) {
    NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;
    m_leftDrive.setNeutralMode(mode);
    m_leftDriveFollower.setNeutralMode(mode);
    m_rightDrive.setNeutralMode(mode);
    m_rightDriveFollower.setNeutralMode(mode);
  }

}
