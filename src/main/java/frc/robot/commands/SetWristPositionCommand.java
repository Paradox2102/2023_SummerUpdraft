// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetWristPositionCommand extends InstantCommand {
  WristSubsystem m_subsystem;
  private double m_setPoint;
  public SetWristPositionCommand(WristSubsystem subsystem, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_setPoint = setPoint;
    Logger.log("SetWristPositionCommand", 0, "SetWristPositionCommand");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.moveSetPoint(m_setPoint);
    Logger.log("SetWristPositionCommand", 1, "initialize");
  }
}
