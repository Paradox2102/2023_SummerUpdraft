// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForTimeCommand extends CommandBase {
  /** Creates a new DriveForTimeCommand. */
  DriveSubsystem m_subsystem;
  private Timer m_timer = new Timer();
  private double m_time;
  private static double k_speed = 2;

  public DriveForTimeCommand(DriveSubsystem subsystem, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    Logger.log("DriveForTimeCommand", 0, "DriveForTimeCommand");
    m_time = time;
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("DriveForTimeCommand", 0, "Initialize");
    m_timer.start();    
    m_timer.reset();
    m_subsystem.setSpeedFPS(k_speed, k_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
    Logger.log("DriveForTimeCommand", 0, "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_time;
  }
}
