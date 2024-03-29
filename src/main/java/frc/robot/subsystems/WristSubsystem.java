// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.dummy.CANSparkMax;
// import com.revrobotics.dummy.RelativeEncoder;
// import com.revrobotics.dummy.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  CANSparkMax m_wristMotor = new CANSparkMax(Constants.Wrist.k_wristMotor, MotorType.kBrushless);
  RelativeEncoder m_wristEncoder = m_wristMotor.getEncoder();
  private static final double k_p = 0.02; //0.075
  private static final double k_i = 0; //0.01;
  private static final double k_f = 0.05;
  private static final double k_d = 0.001;
  private static final double k_maxPower = 0.3;
  private double m_angle = 0;
  private double m_setPoint = 0;
  private boolean m_PIDOn = false;
  private double m_recordedPower = 0;
  private DoubleSupplier m_armAngle;
  private static double k_stallPower = 0.05;
  private static double k_stallSpeed = 0.1;
  private static double k_stallTime = 0.2;
  private Timer m_timer = new Timer();
  PIDController m_wristPID = new PIDController(k_p, k_i, k_d);

  /** Creates a new WristSubsystem. */
  public WristSubsystem(DoubleSupplier armAngle) {
    m_wristMotor.restoreFactoryDefaults();
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristEncoder.setPosition(Constants.Wrist.k_wristStartingAngle);
    m_timer.start();
    m_armAngle = armAngle;
    Logger.log("WristSubsystem", 0, "WristSubsystem");
  }

  public void setPower(double power) {
    switchPID(false);
    m_recordedPower = power;
    m_timer.reset();
    Logger.log("WristSubsystem", 0, String.format("%s, %f", "Set Power: ", m_recordedPower));
  }

  public double getPosition() {
    return m_wristEncoder.getPosition() * Constants.Wrist.k_wristTicksToDegrees;
  }

  public double getPositionForTicks() {
    return m_wristEncoder.getPosition();
  }

  public void moveSetPoint(double setPoint) {
    m_setPoint = setPoint;
    switchPID(true);
    m_timer.reset();
    Logger.log("WristSubsystem", 0, String.format("%s, %f", "Move Set Point: ", m_setPoint));
  }

  public void incrementSetPoint(double increment)
  {
    moveSetPoint(m_setPoint + increment);
  }

  public void switchPID(boolean on) {
    m_PIDOn = on;
    Logger.log("WristSubsystem", 0, String.format("%s, %b", "Switch PID", m_PIDOn));
  }

  private double getFTerm(double angle) {
    m_angle = angle + m_armAngle.getAsDouble();
    double fTerm = (-k_f * Math.sin(Math.toRadians(m_angle)));
    return fTerm;
  }

  @Override
  public void periodic() {
    double power = m_recordedPower;
    // This method will be called once per scheduler run
    //holds the wrist at the set point
    if (m_PIDOn) {
      power = getFTerm(m_setPoint) + m_wristPID.calculate(getPosition(), m_setPoint);
    }
    //set power to 0 if the wrist has been stalled for more than 0.2 seconds
    if (Math.abs(power) > k_stallPower) {
      if (Math.abs(m_wristEncoder.getVelocity()) < k_stallSpeed) {
        if (m_timer.get() > k_stallTime) {
          //Logger.log("WristSubsystem", 0, "Stalled");
          power = 0;
        }
      } else {
        m_timer.reset();
      }
    } 
    if (Math.abs(power) > k_maxPower) {
      power = k_maxPower * Math.signum(power);
    }
    m_wristMotor.set(power);
    SmartDashboard.putNumber("Wrist Power", power);
    SmartDashboard.putNumber("Wrist Speed", m_wristEncoder.getVelocity());
    SmartDashboard.putNumber("Wrist Position In Degrees", getPosition());
    SmartDashboard.putNumber("Wrist Position in Ticks", getPositionForTicks());
  }
}
