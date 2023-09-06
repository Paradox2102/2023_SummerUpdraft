// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HandPosition2 extends InstantCommand {
  private final ArmSubsystem m_armSubsystem;
  private final ReachSubsystem m_reachSubsystem;
  private final WristSubsystem m_wristSubsystem;
  private final double m_armFrontAngle;
  private final double m_armBackAngle;
  private final double m_armExtent;
  private final double m_wristFrontAngle;
  private final double m_wristBackAngle;
  private boolean m_exit = false;
  private final BooleanSupplier m_reverse;

  public HandPosition2(ArmSubsystem armSubsystem, ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem,
      double armFrontAngle, double armBackAngle, double armExtent, double wristFrontAngle, double wristBackAngle, BooleanSupplier reverse) {
    m_armSubsystem = armSubsystem;
    m_reachSubsystem = reachSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_armExtent = armExtent;
    m_armFrontAngle = armFrontAngle;
    m_armBackAngle = armBackAngle;
    m_wristFrontAngle = wristFrontAngle;
    m_wristBackAngle = wristBackAngle;
    m_reverse = reverse;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem, m_reachSubsystem, m_wristSubsystem);
    Logger.log("HandPosition2", 1, "HandPosition2");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("HandPosition2", 1, "Initialize");
    m_armSubsystem.setPosition(m_reverse.getAsBoolean()?m_armFrontAngle:m_armBackAngle);
    // m_reachSubsystem.setPosition(m_armExtent);
    m_wristSubsystem.moveSetPoint(m_reverse.getAsBoolean()?m_wristFrontAngle:m_wristBackAngle);
    m_exit = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_armSubsystem.armOnTarget()) {
      m_reachSubsystem.setPosition(m_armExtent);
      m_exit = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    Logger.log("HandPosition2", 0, "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_exit;
  }
}