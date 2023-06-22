// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

//for stall check
  private static final double k_stallSpeed = 100;
  private static final double k_stallTimer = 0.2;
  private static final double k_stallPower = 0.0;

  private double m_zero;
  private double k_ticsToInches = 29.5/148869;
  private double m_position;
  private double m_reachPower;
//for PID
  private double m_setPoint;
  private boolean m_manual = true;
  private double k_p = 16/30.0;
  private double m_difference;
  

  // enum State {
  //   normal,
  //   stalledUp,
  //   stalledDown
  // }

  // private static State m_state;
  //identify state

  /** Creates a new ReachSubsystem. */
  public ReachSubsystem() {
    // m_state = State.normal;
    m_reachMotor.configFactoryDefault();
    m_zero = m_reachMotor.getSelectedSensorPosition();
  }
  

  public void setPower(double reachPower) {
    m_reachPower = reachPower;
    m_timer.reset();
    m_timer.start();
    m_manual = true;
  }

  public void setPosition(double positionInInches){
    m_setPoint = positionInInches;
    m_timer.reset();
    m_timer.start();
    m_manual = false;
  }

  public double getDistance() {
     m_position = (m_reachMotor.getSelectedSensorPosition() - m_zero) * k_ticsToInches;
     return m_position;
  }

  public double getSpeed() {
    return m_reachMotor.getSelectedSensorVelocity();
  }

  public void setBrakeMode(Boolean brake) {
    NeutralMode mode = brake? NeutralMode.Brake : NeutralMode.Coast;
    m_reachMotor.setNeutralMode(mode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double power;
    double raw = m_reachMotor.getSelectedSensorPosition() - m_zero;

    //P part of the PID
    if(m_manual) {
      power = m_reachPower;
    } else {
      m_difference = getDistance() - m_setPoint;
      power = -k_p * m_difference;
    }

    //stall check
    if (Math.abs(getSpeed()) < k_stallSpeed) {
      if (m_timer.get() > k_stallTimer) {
        // power = Math.copySign(k_stallPower, m_reachPower);
        power = k_stallPower;
      }
    } else {
      m_timer.reset();
    }


    m_reachMotor.set(ControlMode.PercentOutput, power);
    SmartDashboard.putNumber("Cooked Reach position", getDistance());
    SmartDashboard.putNumber("Raw Reach position", raw);
    SmartDashboard.putNumber("Reach Speed", getSpeed());
    SmartDashboard.putNumber("Reach Power", power);
  }
}
    //boolean - dont need true or false (can use directly)

