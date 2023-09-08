// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.DriveSubsystem;

public class CalibrateDriveCommand extends CommandBase {
  private Timer m_timer = new Timer();
  DriveSubsystem m_subsystem;

  public CalibrateDriveCommand(DriveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(subsystem);
    Logger.log("CalibrateDriveCommand", 0, "CalibrateDriveCommand");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("CalibrateDriveCommand", 0, "Initialize");
    m_timer.reset();
    m_timer.start();
    m_subsystem.resetEncoders();
    // m_subsystem.setPower(k_power, k_power);
    m_subsystem.setSpeedFPS(2, 2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Raw Left Motor Pos", m_subsystem.getLeftPosInTicks());
    SmartDashboard.putNumber("Raw Left Motor Speed", m_subsystem.getLeftSpeedInTicks());
    SmartDashboard.putNumber("Raw Right Motor Pos", m_subsystem.getRightPosInTicks());
    SmartDashboard.putNumber("Raw Right Motor Speed", m_subsystem.getRightSpeedInTicks());
    SmartDashboard.putNumber("Time", m_timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
    Logger.log("CalibrateDriveCommand", 0, "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.getLeftPosInTicks() >= 100000;
  }
}
