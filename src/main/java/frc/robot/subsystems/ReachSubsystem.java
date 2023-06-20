// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReachSubsystem extends SubsystemBase {
  private TalonFX m_reachMotor = new TalonFX(Constants.k_reachMotor);
  private Timer m_timer = new Timer();
  private Encoder m_encoder;
  private double m_reachPower;
  private double m_zero;
  private double k_ticksToInches = -29.5/148869;
  private double m_position = 0;
  private static final double k_stallPower = 0;
  private static final double k_stallTime = 2;
  private static final double k_stallSpeed = 1;

  private boolean m_isBrakeMode = false;
  /** Creates a new ReachSubsystem. */
  public ReachSubsystem() {
    m_reachMotor.configFactoryDefault();
    m_zero = m_reachMotor.getSelectedSensorPosition();
  }

  public void setPower(double reachPower) {
    m_reachPower = reachPower;
    m_timer.reset();
    m_timer.start();
  }

  public double getDistance() {
    m_position = (m_reachMotor.getSelectedSensorPosition() - m_zero) * k_ticksToInches;
    return m_position;
  }

 public double getSpeed() {
   return m_reachMotor.getSelectedSensorVelocity();
  }

  public void setBrakeMode(boolean brake) {
    NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;
    m_reachMotor.setNeutralMode(mode);
    m_isBrakeMode = brake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double power = m_reachPower;
    if (Math.abs(getSpeed()) < k_stallSpeed) {
      if (m_timer.get() > k_stallTime) {
        power = k_stallPower;
      } else {
        m_timer.reset();
      }
    }
    m_reachMotor.set(ControlMode.PercentOutput, power);
    SmartDashboard.putNumber("Reach Position", m_position);
    SmartDashboard.putNumber("Reach Speed", getSpeed());
    SmartDashboard.putNumber("Reach Power", power);


  }
}

