// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForTimeCommand extends CommandBase {
  /** Creates a new DriveForTimeCommand. */
  DriveSubsystem m_subsystem;
  private Timer m_timer;
  private double m_time;
  private static double k_speed = 4500;
  public DriveForTimeCommand(DriveSubsystem subsystem, Double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_time = time;
    m_subsystem = subsystem;
    addRequirements(subsystem);
    m_timer.reset();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_subsystem.setSpeed(k_speed, k_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_time;
  }
}
