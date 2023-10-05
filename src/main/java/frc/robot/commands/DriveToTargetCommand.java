// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.ApriltagsCamera.Logger;
import frc.ApriltagsCamera.PositionServer.Target;
import frc.pathfinder.Pathfinder.Path;
import frc.robot.PositionTracker;
import frc.robot.commands.autos.CreatePathCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToTargetCommand extends InstantCommand {
  private final DriveSubsystem m_driveSubsystem;
  private final ArmSubsystem m_armSubsystem;
  private final WristSubsystem m_wristSubsystem;
  private final ReachSubsystem m_reachSubsystem;
  private final PositionTracker m_tracker;
  private DoubleSupplier m_speed;
  private BooleanSupplier m_cancel;
  private BooleanSupplier m_reverse;

  public DriveToTargetCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem,
      ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem, BooleanSupplier reverse, DoubleSupplier speed,
      BooleanSupplier cancel) {
    Logger.log("DriveToTargetCommand", 1, "DriveToTargetCommand()");

    m_driveSubsystem = driveSubsystem;
    m_armSubsystem = armSubsystem;
    m_reachSubsystem = reachSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_tracker = m_driveSubsystem.getTracker();
    m_speed = speed;
    m_cancel = cancel;
    m_reverse = reverse;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("DriveToTargetCommand", 1, "initialize()");

    Target target = m_tracker.m_posServer.getTarget();

    if (target == null) {
      Logger.log("TurnToTargetCommand", 3, "no target");

      return;
    }

    Path path = target.getPath(m_tracker);
    new CreatePathCommand(m_driveSubsystem, path, false, target.isPathReversed(m_tracker), "driveToTarget", m_speed,
        m_cancel).schedule();

    // new SequentialCommandGroup(
    //     new CreatePathCommand(m_driveSubsystem, path, false, target.isPathReversed(m_tracker), "driveToTarget", m_speed,
    //         m_cancel),
    //     new TurnToTargetCommand(m_driveSubsystem, m_armSubsystem, m_reachSubsystem, m_wristSubsystem, m_reverse,
    //         m_cancel))
    //     .schedule();
  }
}
