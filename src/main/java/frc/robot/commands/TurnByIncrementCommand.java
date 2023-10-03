// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PositionTracker;
import frc.robot.subsystems.DriveSubsystem;

public class TurnByIncrementCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private double m_incrementAngleInDegrees;
  private final PositionTracker m_tracker;
  private double m_currentAngle;
  private double m_distance;
  private double m_targetAngleInDegrees = 0;
  private static final double k_angleReset = 5;
  private BooleanSupplier m_cancel;

  /** Creates a new TurnByIncrementCommand. */
  public TurnByIncrementCommand(DriveSubsystem subsystem, double incrementAngleInDegees, BooleanSupplier cancel) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_incrementAngleInDegrees = incrementAngleInDegees;
    m_cancel = cancel;
    m_tracker = m_subsystem.getTracker();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d position = m_tracker.getPose2d();
    m_currentAngle = position.getRotation().getDegrees();
    m_distance = Math.abs(m_currentAngle - m_targetAngleInDegrees);
    if (m_distance > k_angleReset) {
      m_targetAngleInDegrees = m_currentAngle;
    }
    m_targetAngleInDegrees += m_incrementAngleInDegrees;
    new TurnToAngleCommand(m_subsystem, m_targetAngleInDegrees, m_cancel).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
