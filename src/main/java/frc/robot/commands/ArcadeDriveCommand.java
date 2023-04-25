// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//SAVED BUT NOT COMMITED

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  /** Creates a new ArcadeDriveCommand. */
  DriveSubsystem m_subsystem;
  CommandJoystick m_joystick;

  public ArcadeDriveCommand(DriveSubsystem driveSubsystem, CommandJoystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = driveSubsystem;
    m_joystick  = joystick; 

    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = m_joystick.getY();
    double x = m_joystick.getY();

    y = y * y * Math.signum(y);
    x = x * x * Math.signum(x);

    m_subsystem.setPower(y + x, y - x);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
