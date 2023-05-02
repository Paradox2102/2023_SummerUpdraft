// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX m_intakeMotor = new TalonFX(Constants.Intake.k_intakeMotor);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor.configFactoryDefault();
  }

  public void setPower(double intakePower){
    m_intakeMotor.set(ControlMode.PercentOutput, intakePower);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
