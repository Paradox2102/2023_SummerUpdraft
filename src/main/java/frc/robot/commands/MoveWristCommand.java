// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.WristSubsystem;

public class MoveWristCommand extends CommandBase {
  private final WristSubsystem m_subsystem;
  private final double m_power;

  /** Creates a new MoveWristCommand. */
  public MoveWristCommand(WristSubsystem subsystem, Double power) {
    m_subsystem = subsystem;
    m_power = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    Logger.log("MoveWristCommand", 0, "MoveWristCommand");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPower(m_power);
    Logger.log("MoveWristCommand", 0, "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPower(0);
    Logger.log("MoveWristCommand", 0, "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
