// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.ApriltagsCamera.Logger;
import frc.ApriltagsCamera.PositionServer.Target;
import frc.robot.ParadoxField;
import frc.robot.PositionTracker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToTargetCommand extends InstantCommand {
  private final DriveSubsystem m_driveSubsystem;
  private final PositionTracker m_tracker;
  private final ArmSubsystem m_armSubsystem;
  private final ReachSubsystem m_reachSubsystem;
  private final WristSubsystem m_wristSubsystem;
  private final BooleanSupplier m_reverse;
  private final BooleanSupplier m_cancel;

  public TurnToTargetCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem,
      ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem, BooleanSupplier reverse, BooleanSupplier cancel) {
        Logger.log("TurnToTargetCommand", 1, "TurnToTarget");
    m_driveSubsystem = driveSubsystem;
    m_armSubsystem = armSubsystem;
    m_reachSubsystem = reachSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_reverse = reverse;
    m_cancel = cancel;
    m_tracker = m_driveSubsystem.getTracker();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TurnToTargetCommand", 1, "Initialize");
    Pose2d pos = m_tracker.getPose2d();
    double x = pos.getX();
    double y = pos.getY();
    Target target = m_tracker.m_posServer.getTarget();

    if(target == null) {
      Logger.log("TurnToTargetCommand", 3, "no target");

      return;
    }
    double dx = x - target.m_x;
    double dy = y - target.m_y;
    double angleInDegrees = -Math.toDegrees(Math.atan2(dy, dx));
    double dist = 12 * Math.sqrt(dx * dx + dy * dy);

    if (!m_reverse.getAsBoolean()) {
      angleInDegrees = ParadoxField.normalizeAngle(angleInDegrees + 180);
    }

    double armAngle = 90 - Math.toDegrees(Math.atan2(target.m_h, dist));
    double armExtent = Math.sqrt(dist*dist + target.m_h*target.m_h) + target.m_ext;
    double wristAngle = 0;

    if (target.m_no == 9) {
      if (target.m_isCone) {
        wristAngle = -30;
      } else {
        wristAngle = -145;
      }
    } else {
      // reg positions
      switch (target.m_level) {
        case 0: // low position
          if (target.m_isCone) {
            wristAngle = -95;
          } else {
            wristAngle = -18;
          }
          break;
        case 1: // middle
          if (target.m_isCone) {
            wristAngle = -100;
          } else {
            wristAngle = -117;
          }
          break;
        case 2: // high
          if (target.m_isCone) {
            wristAngle = -95;
          } else {
            wristAngle = -109;
          }
          break;
      }
    }

    new TurnToAngleCommand(m_driveSubsystem, angleInDegrees, m_cancel).schedule();

    // new SequentialCommandGroup(
    //    new TurnToAngleCommand(m_driveSubsystem, angleInDegrees, m_cancel),
    //    new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, armAngle, -armAngle,
    //        armExtent, wristAngle, -wristAngle, m_reverse))
    //    .schedule();
  }
}
