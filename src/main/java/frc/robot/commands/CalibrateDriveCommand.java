// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class CalibrateDriveCommand extends CommandBase {
  /** Creates a new TestSpeedCommand. */
  //private static double k_power = 0.5;
  //private static double k_speed = 5;
  private static double k_maxSpeed = Constants.Drive.k_maxSpeed;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setSpeed((k_maxSpeed/2), (k_maxSpeed/2));
    //m_subsystem.setPower(k_power, k_power);
    //m_subsystem.setSpeedFPS(k_speed, k_speed);
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
    return false;
  }
}
