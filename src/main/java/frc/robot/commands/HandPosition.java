// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HandPosition extends InstantCommand {
  private final ArmSubsystem m_armSubsystem;
  private final ReachSubsystem m_reachSubsystem;
  private final WristSubsystem m_wristSubsystem;
  private final double m_armAngle;
  private final double m_armExtent;
  private final double m_wristAngle;

  public HandPosition(ArmSubsystem armSubsystem, ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem, double armAngle, double armExtent, double wristAngle) {
    m_armSubsystem = armSubsystem;
    m_reachSubsystem = reachSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_armAngle = armAngle;
    m_armExtent = armExtent;
    m_wristAngle = wristAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem, m_reachSubsystem, m_wristSubsystem);
  Logger.log("HandPosition", 1, "HandPosition"); 
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("HandPosition", 1, "Initialize");
    m_armSubsystem.setPosition(m_armAngle);
    m_reachSubsystem.setPosition(m_armExtent);
    m_wristSubsystem.moveSetPoint(m_wristAngle);
    }
}
