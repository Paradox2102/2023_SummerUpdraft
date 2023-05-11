// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  CANSparkMax m_wristMotor = new CANSparkMax(Constants.Wrist.k_wristMotor, MotorType.kBrushless);
  RelativeEncoder m_wristEncoder = m_wristMotor.getEncoder();
  private static final double k_p = 0.075;
  private static final double k_i = 0;
  private static final double kf = 0.01;
  private double m_angle = 0;
  private double m_setPoint = 0;
  private boolean m_PIDOn = false;
  PIDController m_wristPID = new PIDController(k_p, k_i, 0);

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    m_wristMotor.restoreFactoryDefaults();
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristEncoder.setPosition(Constants.Wrist.k_wristStartingAngle);
  }

  public void setPower(double power, boolean PID) {
    m_wristMotor.set(power);
    switchPID(PID);
  }

  public double getPosition() {
    return m_wristEncoder.getPosition() * Constants.Wrist.k_wristTicksToDegrees;
  }

  public void moveSetPoint(double setPoint) {
    m_setPoint = setPoint;
    switchPID(true);
  }

  public void switchPID(boolean on) {
    m_PIDOn = on;
  }

  private double getFTerm(double angle) {
    m_angle = angle;
    double fTerm = (-kf * Math.sin(Math.toRadians(m_angle)));
    return fTerm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_PIDOn) {
      double power = getFTerm(m_setPoint) + m_wristPID.calculate(getPosition(), m_setPoint);
      setPower(power, true);
      SmartDashboard.putNumber("Wrist Power", power);
    }
    SmartDashboard.putNumber("Wrist Position In Degrees", getPosition());
  }
}