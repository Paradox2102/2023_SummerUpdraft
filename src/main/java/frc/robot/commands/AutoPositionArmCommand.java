// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.ApriltagsCamera.Logger;
import frc.ApriltagsCamera.PositionServer;
import frc.ApriltagsCamera.PositionServer.Target;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPositionArmCommand extends InstantCommand {
  private final DriveSubsystem m_subsystem;
  private final ArmSubsystem m_armSubsystem;
  private final ReachSubsystem m_reachSubsystem;
  private final WristSubsystem m_wristSubsystem;
  private final BooleanSupplier m_reverse;
  private final PositionServer m_server;

  public AutoPositionArmCommand(DriveSubsystem subsystem, BooleanSupplier reverse, ArmSubsystem armSubsystem,
      ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem) {
    Logger.log("AutoPositionArmCommand", 0, "AutoPositionArmCommand");
    m_subsystem = subsystem;
    m_armSubsystem = armSubsystem;
    m_reachSubsystem = reachSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_reverse = reverse;
    m_server = m_subsystem.getTracker().m_posServer;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("AutoPositionArmCommand", 0, "Initialize");
    Target target = m_server.getTarget();
    if (target != null) {
      HandPosition2 command = null;
      Logger.log("AutoPositionArmCommand", 0,
          String.format("level = %d, cone = %b,no = %d", target.m_level, target.m_isCone, target.m_no));
      // feeder positions
      if (target.m_no == 9) {
        if (target.m_isCone) {
          command = new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 47, -47, 0, -30, 30,
              m_reverse);
        } else {
          command = new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 22, -22, 9, -145, 145,
              m_reverse);
        }
      } else {
        // reg positions
        switch (target.m_level) {
          case 0: // low position
            if (target.m_isCone) {
              command = new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 42, -42, 1, -95, 85,
                  m_reverse);
            } else {
              command = new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 103, -96, 0, -18, 15,
                  m_reverse);
            }
            break;
          case 1: // middle
            if (target.m_isCone) {
              command = new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 45, -45, 14, -100, 100,
                  m_reverse);
            } else {
              command = new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 40, -40, 10, -117, 117,
                  m_reverse);
            }
            break;
          case 2: // high
            if (target.m_isCone) {
              command = new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 42, -42,
                  Constants.Reach.k_maxReach, -95, 85,
                  m_reverse);
            } else {
              command = new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 47, -47, 27, -109, 109,
                  m_reverse);
            }
            break;
        }

      }

      if (command != null) {
        command.schedule();
      }

    } else {
      Logger.log("AutoPositionArmCommand", 0, "No Target");
    }
  }
}
