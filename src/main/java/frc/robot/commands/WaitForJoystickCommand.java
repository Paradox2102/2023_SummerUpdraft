// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class WaitForJoystickCommand extends CommandBase {
  /** Creates a new WaitForJoystickCommand. */
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_speed;

  public WaitForJoystickCommand(DriveSubsystem subsystem, DoubleSupplier speed) {
    m_driveSubsystem = subsystem;
    m_speed = speed;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_speed.getAsDouble()) < 0.1);
  }
}
