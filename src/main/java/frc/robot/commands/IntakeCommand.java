// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  IntakeSubsystem m_subsystem;
  double m_power;

  public IntakeCommand(IntakeSubsystem subsystem, double power) {
    m_subsystem = subsystem;
    m_power = power;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
    Logger.log("IntakeCommand", 0, "IntakeCommand");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPower(m_power);
    Logger.log("IntakeCommand", 0, "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPower(0);
    Logger.log("IntakeCommand", 0, "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
