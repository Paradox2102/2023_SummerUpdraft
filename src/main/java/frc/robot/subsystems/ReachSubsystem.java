// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.dummy.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants;

public class ReachSubsystem extends SubsystemBase {
  private TalonFX m_reachMotor = new TalonFX(Constants.k_reachMotor);
  private Timer m_timer = new Timer();
  private double m_reachPower = 0;
  private double m_zero;
  private double m_difference;
  private double m_setPosition;
  private boolean m_manual = true;
  private double k_ticksToInches = 29.5 / 148869;
  private double m_position = 0;
  private static final double k_stallPower = 0.1;
  private static final double k_stallTime = 1;
  private static final double k_stallSpeed = 250;
  private static final double k_p = 16 / 30.0;
  private static final double k_f = 0.06;
  private DoubleSupplier m_getArmAngleInDegrees;

  enum State {
    normal,
    stalledUp,
    stalledDown
  }

  private static State m_state = State.normal;
  // identify state

  /** Creates a new ReachSubsystem. */
  public ReachSubsystem(DoubleSupplier armAngleInDegrees) {
    m_state = State.normal;
    m_reachMotor.configFactoryDefault();
    m_zero = m_reachMotor.getSelectedSensorPosition();
    setBrakeMode(true);
    m_reachMotor.setInverted(true);
    m_getArmAngleInDegrees = armAngleInDegrees;
    Logger.log("ReachSubsystem", 0, "ReachSubsystem");
  }

  public void setPower(double reachPower) {
    m_reachPower = reachPower;
    m_timer.reset();
    m_timer.start();
    m_manual = true;
    Logger.log("ReachSubsystem", 0, String.format("%s, %f", "Set Power", m_reachPower));
  }

  public void setPosition(double positionInInches) {
    m_setPosition = positionInInches;
    m_timer.reset();
    m_timer.start();
    m_manual = false;
    m_state = State.normal;
  }

  public void incrementPosition(double incInInches) {
    setPosition(m_setPosition + incInInches);
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
    SmartDashboard.putBoolean("Reach Brake", brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double power = m_reachPower;
    double raw = m_reachMotor.getSelectedSensorPosition() - m_zero;
    // move manually or move to position (speed depending on distance)
    if (m_manual) {
      power = m_reachPower;
    } else {
      m_difference = getDistance() - m_setPosition;
      // current-set;
      power = -k_p * m_difference + k_f * Math.cos(Math.toRadians(m_getArmAngleInDegrees.getAsDouble()));
      // if (Math.abs(power) > k_maxPower) {
      // power = k_maxPower * Math.signum(power);
      // }
    }
    SmartDashboard.putNumber("Reach Raw Power", power);

    // stall check
    if (Math.abs(power) > k_stallPower) {
      if (Math.abs(getSpeed()) < k_stallSpeed) {
        // Logger.log("Reach Subsystem", 1, String.format("SPEEEEED = %f", getSpeed()));
        if (m_timer.get() > k_stallTime) {
          // Logger.log("Reach Subsystem", 1, String.format("power = %f", power));
          if (power > 0) {
            m_state = State.stalledUp;
          } else {
            m_state = State.stalledDown;
            // Logger.log("Reach Subsystem", 1, String.format("state = %s",
            // m_state.toString()));
          }
        } else {
          m_timer.reset();
        }
      }
    }

    // state change
    if (m_state == State.stalledUp && power < 0) {
      m_state = State.normal;
    }
    if (m_state == State.stalledDown && power > 0) {
      m_state = State.normal;
    }

    if (m_state == State.stalledUp && power > 0) {
      power = 0;
    }
    if (m_state == State.stalledDown && power < 0) {
      power = 0;
    }
    m_reachMotor.set(ControlMode.PercentOutput, power);
    SmartDashboard.putNumber("Cooked Reach Position", getDistance());
    SmartDashboard.putNumber("Raw Reach Position", raw);
    SmartDashboard.putNumber("Reach Speed", getSpeed());
    SmartDashboard.putNumber("Reach Power", power);
    SmartDashboard.putString("Reach State", m_state.toString());

  }
}
