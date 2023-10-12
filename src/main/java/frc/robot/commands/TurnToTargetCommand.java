// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.ApriltagsCamera.Logger;
import frc.ApriltagsCamera.PositionServer.Target;
import frc.robot.Constants;
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
  // private final PositionTracker m_tracker;
  private final ArmSubsystem m_armSubsystem;
  private final ReachSubsystem m_reachSubsystem;
  private final WristSubsystem m_wristSubsystem;
  // private final BooleanSupplier m_reverse;
  private final BooleanSupplier m_cancel;
  static private final double k_minArmLength = 27.0;
  private final boolean m_turnOnly;
  private final DoubleSupplier m_speed;

  public TurnToTargetCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem,
      ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem,
      /* BooleanSupplier reverse, */ BooleanSupplier cancel, boolean turnOnly, DoubleSupplier speed) {
    Logger.log("TurnToTargetCommand", 1, "TurnToTarget");
    m_driveSubsystem = driveSubsystem;
    m_armSubsystem = armSubsystem;
    m_reachSubsystem = reachSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_turnOnly = turnOnly;
    m_speed = speed;
    // m_reverse = reverse;
    m_cancel = cancel;
    // m_tracker = m_driveSubsystem.getTracker();
  }

  // Compute the commands needed to turn to the target and deploy the arm
  // Returns an array:
  // commands[0] = command to turn to target
  // commands[1] = command to deploy the arm
  public static Command[] getCommands(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem,
      ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem, BooleanSupplier cancel) {

    Logger.log("TurnToTargetCommand", 1, "getCommands");
    PositionTracker tracker = driveSubsystem.getTracker();
    Pose2d pos = tracker.getPose2d();
    double x = pos.getX();
    double y = pos.getY();
    Target target = tracker.m_posServer.getTarget();

    if (target == null) {
      Logger.log("TurnToTargetCommand", 3, "no target");

      return null;
    }
    double dx = x - target.m_x;
    double dy = y - target.m_y;
    double angleInDegrees = -Math.toDegrees(Math.atan2(dy, dx));
    double dist = 12 * Math.sqrt(dx * dx + dy * dy);

    // if (!m_reverse.getAsBoolean()) {
    if (target.isPathReversed(tracker)) {
      angleInDegrees = ParadoxField.normalizeAngle(angleInDegrees + 180);
    }
    {
      angleInDegrees = ParadoxField.normalizeAngle(-180 - angleInDegrees);
    }

    double armAngle = 90 - Math.toDegrees(Math.atan2(target.m_h, dist));
    double armExtent = Math.sqrt(dist * dist + target.m_h * target.m_h) + target.m_ext - k_minArmLength - 5;
    if (armExtent > Constants.Reach.k_maxReach) {
      armExtent = Constants.Reach.k_maxReach;
    } else if (armExtent < 0) {
      armExtent = 0;
    }
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
            wristAngle = -100;
          }
          break;
      }
    }
    SmartDashboard.putNumber("TT AngleInDegrees", angleInDegrees);
    SmartDashboard.putNumber("TT Distance", dist);
    SmartDashboard.putNumber("TT ArmAngle", armAngle);
    SmartDashboard.putNumber("TT ArmExtent", armExtent);
    SmartDashboard.putNumber("TT WristAngle", wristAngle);
    SmartDashboard.putBoolean("TT Reverse", target.isPathReversed(tracker));
    SmartDashboard.putNumber("TT TargetHeight", target.m_h);
    SmartDashboard.putNumber("TT Height", target.m_h);
    SmartDashboard.putNumber("TT X", target.m_x);
    SmartDashboard.putNumber("TT Y", target.m_y);
    SmartDashboard.putNumber("TT ext", target.m_ext);

    return new Command[] { new TurnToAngleCommand(driveSubsystem, angleInDegrees, cancel),
        new HandPosition2(armSubsystem, reachSubsystem, wristSubsystem, armAngle, -armAngle,
            armExtent, wristAngle, -wristAngle, () -> target.isPathReversed(tracker)) };
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TurnToTargetCommand", 1, "Initialize");

    Command[] commands = getCommands(m_driveSubsystem, m_armSubsystem, m_reachSubsystem, m_wristSubsystem, m_cancel);

    if (commands != null) {
      if (m_turnOnly) {
        new SequentialCommandGroup(commands[0], new WaitForJoystickCommand(m_driveSubsystem, m_speed)).schedule();
      } else {
        new SequentialCommandGroup(commands[0], commands[1]).schedule();
      }
    }
  }
}
