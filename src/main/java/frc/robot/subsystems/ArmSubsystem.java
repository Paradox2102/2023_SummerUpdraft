// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax m_armMotor = new CANSparkMax(Constants.Arm.k_armMotor, MotorType.kBrushless);
  CANSparkMax m_armFollower = new CANSparkMax(Constants.Arm.k_armFollower, MotorType.kBrushless);

  RelativeEncoder m_armRelative = m_armMotor.getEncoder();

  Solenoid m_brake = new Solenoid(PneumaticsModuleType.REVPH, Constants.Arm.k_armBrake);

  private double m_recordedPower;
  private Timer m_timer = new Timer();
  private static double k_stallPower = 0.05;
  private static double k_stallSpeed = 100;
  private static double k_stallTime = 0.2;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor.restoreFactoryDefaults();
    m_armFollower.restoreFactoryDefaults();
    m_armFollower.follow(m_armMotor, true);
    m_armMotor.setIdleMode(IdleMode.kBrake);
    m_armFollower.setIdleMode(IdleMode.kBrake);
    m_timer.start();
    SetArmBrake(false);
  }

  public void SetPower(double power){
    //m_armMotor.set(power);
    m_recordedPower = power;
    m_timer.reset();
  }

  public void SetArmBrake(boolean brake) {
    m_brake.set(!brake);
    SmartDashboard.putBoolean("Arm Brake", brake);
  }

  @Override
  public void periodic() {
    double power = m_recordedPower;
    if(power > k_stallPower){
      if(Math.abs(m_armRelative.getVelocity()) < k_stallSpeed){
        if(m_timer.get() > k_stallTime){
          power = 0;
        }
      } else {
        m_timer.reset();
      }
    }
    m_armMotor.set(power);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Speed", m_armRelative.getVelocity());
    SmartDashboard.putNumber("Arm Power", power);
  }
}
