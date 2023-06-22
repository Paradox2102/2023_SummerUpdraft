// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX m_armMotor = new TalonFX(Constants.Arm.k_armFollower);
  private TalonFX m_armFollower = new TalonFX(Constants.Arm.k_armMotor);
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(TalonFX armMotor) {
    m_armMotor = armMotor;
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
