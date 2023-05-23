// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCommand extends CommandBase {
  private final ArmSubsystem m_subsystem;
  private double m_power;
  /** Creates a new MoveArmCommand. */
  public MoveArmCommand(ArmSubsystem subsystem, Double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.SetPower(m_power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.SetPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
