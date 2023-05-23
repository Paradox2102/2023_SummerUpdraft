// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private final Joystick m_joystick;

  /** Creates a new ArcadeDrive. */
  public ArcadeDriveCommand(DriveSubsystem subsystem, Joystick joystick) {
    // Logger.log("ArcadeDriveCommand", 3, "ArcadeDriveCommand()");

    m_subsystem = subsystem;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double y = m_joystick.getY();
    double x = m_joystick.getX();

    x = Math.abs(x) * x;
    y = Math.abs(y) * y; 

    m_subsystem.setPower(y + x, y - x);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
