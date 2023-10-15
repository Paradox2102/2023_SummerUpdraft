// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.dummy.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX m_intakeMotor = new TalonFX(Constants.Intake.k_intakeMotor);
  private Timer m_timer = new Timer();
  private static double k_expiredTimer = 0.1;
  private double m_savedPower;
  private static double k_stallSpeed = 2000;//3000
  private static double k_stallPower = 0.1; //0.075;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor.configFactoryDefault();
    Logger.log("IntakeSubsystem", 0, "IntakeSubsystem");
  }

  public void setPower(double intakePower){
    m_savedPower = intakePower;
    m_timer.reset();
    m_timer.start();
    Logger.log("IntakeSubsystem", 0, String.format("%s, %f", "Set Power: ", m_savedPower));
  }
  public double getSpeed(){
    return m_intakeMotor.getSelectedSensorVelocity();   
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Speed", getSpeed());

    double power = m_savedPower;
    // if(Math.abs(getSpeed()) < k_stallSpeed && m_savedPower != 0) {
    //   if(m_timer.get() > k_expiredTimer) {
    //     //Logger.log("IntakeSubsystem", 0, "Stalled");
    //     if(m_savedPower < 0) {
    //       power = -k_stallPower;
    //     }  else {
    //       power = k_stallPower;
    //     }
    //   } 
    // } else {
    //   m_timer.reset();
    // }
    m_intakeMotor.set(ControlMode.PercentOutput, power);

    SmartDashboard.putNumber("Intake Power", power);
  }

    //in subsystem (lowest level)
    //setpower -> record recquied power
    //encoder, 
    //timer, then resist timer after __
    //after timer is done, set power to optimal "stall" speed
}

