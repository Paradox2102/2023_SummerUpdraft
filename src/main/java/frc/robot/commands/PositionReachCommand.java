// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ReachSubsystem;

public class PositionReachCommand extends CommandBase {
  ReachSubsystem m_subsystem;
  double m_setPosition;
  /** Creates a new PositionReachCommand. */
  public PositionReachCommand(ReachSubsystem subsystem, double positionInInches ) {
    m_subsystem = subsystem;
    m_setPosition = positionInInches;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
    Logger.log("PositionReachCommand", 0, "PositionReachCommand");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPosition(m_setPosition);
    Logger.log("PositionReachCommand", 0, "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("PositionReachCommand", 0, "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
