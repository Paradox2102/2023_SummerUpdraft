// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax m_armMotor = new CANSparkMax(Constants.Arm.k_armMotor, MotorType.kBrushless);
  CANSparkMax m_armFollower = new CANSparkMax(Constants.Arm.k_armFollower, MotorType.kBrushless);

  TalonSRX m_armEncoder = new TalonSRX(Constants.Arm.k_armEncoder);
  RelativeEncoder m_armRelative = m_armMotor.getEncoder();

  Solenoid m_brake = new Solenoid(PneumaticsModuleType.REVPH, Constants.Arm.k_armBrake);

  PIDController m_armPID = new PIDController(k_p, k_i, k_d);
  private Boolean m_PIDOn = false;
  private Double m_setPoint = 0.0;

  private double m_recordedPower;
  private Timer m_timer = new Timer();
  private static double k_stallPower = 0.05;
  private static double k_stallSpeed = 100;
  private static double k_stallTime = 0.2;
  private static double k_armDeadZone = 2;
  private static double k_p = 0.02;
  private static double k_i = 0;
  private static double k_d = 0.001;
  private static double k_f = 0.005;
  private static double k_l = 18;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor.restoreFactoryDefaults();
    m_armFollower.restoreFactoryDefaults();
    m_armFollower.follow(m_armMotor, true);
    m_armMotor.setIdleMode(IdleMode.kBrake);
    m_armFollower.setIdleMode(IdleMode.kBrake);
    m_armEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    m_timer.start();
    setArmBrake(true);
  }

  public void setPower(double power){
    //m_armMotor.set(power);
    m_recordedPower = power;
    m_PIDOn = false;
    m_timer.reset();
  }

  public void setPosition(double setPoint){
    m_setPoint = setPoint;
    m_PIDOn = true;
    m_timer.reset();
    if (!armOnTarget()){
      setArmBrake(false);
    }
  }

  public void setArmBrake(boolean brake) {
    m_brake.set(!brake);
    SmartDashboard.putBoolean("Arm Brake", brake);
  }

  public double getArmAngleDegrees() {
    return m_armEncoder.getSelectedSensorPosition() * Constants.Arm.k_armTicksToDegrees - Constants.Arm.k_armZeroAngle;
  }

  public double getFTerm(double angleInDegrees) {
    double fTerm = ((-k_f * k_l) * Math.sin(Math.toRadians(angleInDegrees)));
    return fTerm;
  }

  public boolean armOnTarget() {
    return Math.abs(m_setPoint - getArmAngleDegrees()) < k_armDeadZone;
  }

  @Override
  public void periodic() {
    double power = m_recordedPower;
    if (m_PIDOn) {
      power = getFTerm(m_setPoint) + m_armPID.calculate(getArmAngleDegrees(), m_setPoint);
    } 
    if (power > k_stallPower){
      if(Math.abs(m_armRelative.getVelocity()) < k_stallSpeed){
        if(m_timer.get() > k_stallTime){
          power = 0;
        }
      } else {
        m_timer.reset();
      }
    }
    m_armMotor.set(power);
    if (armOnTarget()){
      setArmBrake(true);
    }
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Speed", m_armRelative.getVelocity());
    SmartDashboard.putNumber("Arm Power", power);
    SmartDashboard.putNumber("Arm Angle In Degrees", getArmAngleDegrees());
    SmartDashboard.putNumber("Arm Angle Raw", m_armEncoder.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Set Point", m_setPoint);
  }
}
