// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.FilenameFilter;

import javax.swing.text.StyledEditorKit.FontFamilyAction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReachSubsystem extends SubsystemBase {
  private TalonFX m_reachMotor = new TalonFX(Constants.Reach.k_reachMotor);
  private Timer m_timer = new Timer();
  private static final double k_stallSpeed = 1500;
  private static final double k_stallTimer = 0.2;
  private double m_reachPower;

  enum State {
    normal,
    stalledUp,
    stalledDown
  }

  private static State m_state;
  //identify state

  /** Creates a new ReachSubsystem. */
  public ReachSubsystem() {
    m_state = State.normal;
    m_reachMotor.configFactoryDefault();
  }

  public void setPower(double reachPower) {
    m_reachPower = reachPower;
    m_timer.reset();
    m_timer.start();
  }

  public double getSpeed() {
    return m_reachMotor.getSelectedSensorVelocity();
  }

  public void setBrakeMode(Boolean brake) {
    NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;
    m_reachMotor.setNeutralMode(mode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double power = m_reachPower;
    if (Math.abs(getSpeed()) < k_stallSpeed) {
      if (m_timer.get() > k_stallTimer) {
        power = 0;
      }
    } else {
      m_timer.reset();
    }
    m_reachMotor.set(ControlMode.PercentOutput, power);
    SmartDashboard.putNumber("Reach Speed", getSpeed());
    SmartDashboard.putNumber("Reach Power", power);
  }
}
