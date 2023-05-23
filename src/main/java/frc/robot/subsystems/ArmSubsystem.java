// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax m_armMotor = new CANSparkMax(Constants.Arm.k_armMotor, MotorType.kBrushless);
  CANSparkMax m_armFollower = new CANSparkMax(Constants.Arm.k_armFollower, MotorType.kBrushless);
  private double m_recordedPower;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor.restoreFactoryDefaults();
    m_armFollower.restoreFactoryDefaults();
    m_armFollower.follow(m_armMotor, true);
    m_armMotor.setIdleMode(IdleMode.kBrake);
    m_armFollower.setIdleMode(IdleMode.kBrake);
  }

  public void SetPower(double power){
    m_armMotor.set(power);
    m_recordedPower = power;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Power",m_recordedPower);
  }
}
