// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveUntilPitchCommand extends CommandBase {
  private DriveSubsystem m_subsystem;
  private double m_pitch;
  private static double k_speed = 5;
  /** Creates a new DriveUntilPitchCommand. */
  public DriveUntilPitchCommand(DriveSubsystem subsystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_pitch = angle;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setSpeedFPS(k_speed, k_speed);
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
    return Math.abs(m_subsystem.getPitchInDegrees()) >= m_pitch;
  }
}
