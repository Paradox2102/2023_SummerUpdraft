// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.ApriltagsCamera.Logger;
import frc.ApriltagsCamera.PositionServer.Target;
import frc.pathfinder.Pathfinder.Path;
import frc.robot.PositionTracker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToTargetCommand extends InstantCommand {
  private final DriveSubsystem m_driveSubsystem;
  private final PositionTracker m_tracker;

  public DriveToTargetCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem,
              ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem, BooleanSupplier reverse) {
    Logger.log("DriveToTargetCommand", 1, "DriveToTargetCommand()");

    m_driveSubsystem = driveSubsystem;
    m_tracker = m_driveSubsystem.getTracker();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("DriveToTargetCommand", 1, "initialize()");

    Target target = m_tracker.m_posServer.getTarget();

    if(target == null) {
      Logger.log("TurnToTargetCommand", 3, "no target");

      return;
    }

    Path path = target.getPath(m_tracker);
  }
}
