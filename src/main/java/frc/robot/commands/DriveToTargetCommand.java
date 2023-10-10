// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

public class DriveToTargetCommand extends InstantCommand {
  private final DriveSubsystem m_driveSubsystem;
  private final ArmSubsystem m_armSubsystem;
  private final WristSubsystem m_wristSubsystem;
  private final ReachSubsystem m_reachSubsystem;
  private final PositionTracker m_tracker;
  private DoubleSupplier m_speed;
  private BooleanSupplier m_cancel;
  // private BooleanSupplier m_reverse;

  public DriveToTargetCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem,
      ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem, /* BooleanSupplier reverse, */ DoubleSupplier speed,
      BooleanSupplier cancel) {
    Logger.log("DriveToTargetCommand", 1, "DriveToTargetCommand()");

    m_driveSubsystem = driveSubsystem;
    m_armSubsystem = armSubsystem;
    m_reachSubsystem = reachSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_tracker = m_driveSubsystem.getTracker();
    m_speed = speed;
    m_cancel = cancel;
    // m_reverse = reverse;
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

    SmartDashboard.putBoolean("Path reversed", target.isPathReversed(m_tracker));

    @SuppressWarnings("unused")
    Command commands[] = TurnToTargetCommand.getCommands(m_driveSubsystem, m_armSubsystem, m_reachSubsystem,
        m_wristSubsystem, m_cancel);

    if (commands != null) {
      if (target.m_no == 9) {
        // Just drive to target, do not turn or deploy arm
        new SequentialCommandGroup(
            new CreatePathCommand(m_driveSubsystem, path, false,
                target.isPathReversed(m_tracker),
                "driveToTarget", 0.5,
                // null, null),
                m_speed, m_cancel),
            new WaitForJoystickCommand(m_driveSubsystem, m_speed)).schedule();
      } else {
        // Un-comment to get it to turn to target at the end
        new SequentialCommandGroup(
            new CreatePathCommand(m_driveSubsystem, path, false, target.isPathReversed(m_tracker),
                "driveToTarget", 0.5,
                // null, null),
                m_speed, m_cancel),
            new TurnToTargetCommand(m_driveSubsystem, m_armSubsystem, m_reachSubsystem, m_wristSubsystem, m_cancel, true, m_speed)).schedule();
            // commands[0],
            // new WaitForJoystickCommand(m_driveSubsystem, m_speed)).schedule();
      }

      // Uncomment to get it to turn to target and deploy arm at end
      // new SequentialCommandGroup(
      // new CreatePathCommand(m_driveSubsystem, path, false,
      // target.isPathReversed(m_tracker),
      // "driveToTarget", 0.5,
      // // null, null),
      // m_speed, m_cancel),
      // commands[0], commands[1],
      // new WaitForJoystickCommand(m_driveSubsystem, m_speed)).schedule();
    }
  }
}
