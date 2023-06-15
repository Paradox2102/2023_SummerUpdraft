// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_rightDrive = new WPI_TalonFX(Constants.Drive.k_rightDrive);
  private final WPI_TalonFX m_rightFollower = new WPI_TalonFX(Constants.Drive.k_rightFollower);
  private final WPI_TalonFX m_leftDrive = new WPI_TalonFX(Constants.Drive.k_leftDrive);
  private final WPI_TalonFX m_leftFollower = new WPI_TalonFX(Constants.Drive.k_leftFollower);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_rightDrive.configFactoryDefault();
    m_rightFollower.configFactoryDefault();
    m_leftDrive.configFactoryDefault();
    m_leftFollower.configFactoryDefault();
    m_rightDrive.setInverted(TalonFXInvertType.CounterClockwise);
    m_rightFollower.follow(m_rightDrive);
    m_rightFollower.setInverted(TalonFXInvertType.FollowMaster);
    m_leftDrive.setInverted(TalonFXInvertType.Clockwise);
    m_leftFollower.follow(m_leftDrive);
    m_leftFollower.setInverted(TalonFXInvertType.FollowMaster);
    Logger.log("DriveSubsystem", 0, "DriveSubsystem");
  }

  public void setPower(double rightPower, double leftPower) {
    m_rightDrive.set(TalonFXControlMode.PercentOutput, rightPower);
    m_leftDrive.set(TalonFXControlMode.PercentOutput, leftPower);
    Logger.log("DriveSubsystem", 0, String.format("%s, %f, %s, %f", "Set Power Left: ", leftPower, " Right: ", rightPower));
  }

  public void stop() {
    setPower(0, 0);
    Logger.log("DriveSubsystem", 0, "Stop");
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  public void setBrake(boolean brake) {
    if (brake) {
      m_rightDrive.setNeutralMode(NeutralMode.Brake);
      m_rightFollower.setNeutralMode(NeutralMode.Brake);
      m_leftDrive.setNeutralMode(NeutralMode.Brake);
      m_leftFollower.setNeutralMode(NeutralMode.Brake);
      Logger.log("DriveSubsystem", 0, "Brake Mode");
    } else {
      m_rightDrive.setNeutralMode(NeutralMode.Coast);
      m_rightFollower.setNeutralMode(NeutralMode.Coast);
      m_leftDrive.setNeutralMode(NeutralMode.Coast);
      m_leftFollower.setNeutralMode(NeutralMode.Coast);
      Logger.log("DriveSubsystem", 0, "Coast Mode");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
