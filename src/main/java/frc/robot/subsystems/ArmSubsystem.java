// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // what do all of these do and why is there an error?

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  //what is this???
  Solenoid m_brake = new Solenoid(PneumaticsModuleType.REVPH, Constants.k_armBrake);

  // arm motors
  CANSparkMax m_arm = new CANSparkMax(Constants.k_armMotor, MotorType.kBrushless); // what is motor type?
  CANSparkMax m_armFollower = new CANSparkMax(Constants.k_armFollower, MotorType.kBrushless);

  //encoder
  RelativeEncoder m_armEncoder = m_arm.getEncoder(); // why does it need to say relative?

  //limit switches
  private SparkMaxLimitSwitch m_armForwardLimit;
  private SparkMaxLimitSwitch m_armReverseLimit;

  public ArmSubsystem() {
    //reset motors
    m_arm.restoreFactoryDefaults();
    m_armFollower.restoreFactoryDefaults();
    m_armFollower.follow(m_arm, true);

    //set limit switches
    m_armForwardLimit = m_arm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_armReverseLimit = m_arm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_armForwardLimit.enableLimitSwitch(true);
    m_armReverseLimit.enableLimitSwitch(true);

    //encoder (ticks to degrees)
    m_armEncoder.setPositionConversionFactor(Constants.k_armTicksToDegrees);

    //set brake
    m_arm.setIdleMode(IdleMode.kBrake);
    m_armFollower.setIdleMode(IdleMode.kBrake);
    setArmBrake(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }

  //what does this do?
  public void setArmBrake(boolean brake) {
    m_brake.set(!brake);
    SmartDashboard.putNumber("Arm Brake", brake ? 1 : 0);
  }
}
