// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.ParadoxField;
import frc.robot.PositionTracker;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private final double m_targetAngleInDegrees;
  private final PositionTracker m_tracker;
  private final BooleanSupplier m_cancel;
  //private static final double k_minPower = 0.07;
  private static final double k_minSpeed = 0.8; //0.4;
  private static final double k_maxSpeed = 4;
  private static final double k_deadZone = 1;
  public static final double k_p = 12.0/180;


  /** Creates a new TurnToAngleCommand. */
  public TurnToAngleCommand(DriveSubsystem subsystem, double targetAngleInDegrees, BooleanSupplier cancel) {
    Logger.log("TurnToAngleCommand", 0, "TurnToAngleCommand()");
    
    m_subsystem = subsystem;
    m_targetAngleInDegrees = targetAngleInDegrees;
    m_tracker = m_subsystem.getTracker();
    m_cancel = cancel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TurnToAngleCommand", 0, "Initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d position = m_tracker.getPose2d();
    double normDifference = getAngleError();
    double speed = k_p * normDifference;
    if (Math.abs(speed) < k_minSpeed) {
      speed = k_minSpeed * Math.signum(speed);
    }
    else if (Math.abs(speed) > k_maxSpeed)
    {
      speed = k_maxSpeed * Math.signum(speed);
    }

    m_subsystem.setSpeedFPS(speed, -speed);
    SmartDashboard.putNumber("target angle", m_targetAngleInDegrees);
    SmartDashboard.putNumber("robot angle", position.getRotation().getDegrees());
    SmartDashboard.putNumber("turn speed", speed);
    Logger.log("TurnToAngleCommand", 1, String.format("execute: t=%f,r=%f,s=%f", m_targetAngleInDegrees, position.getRotation().getDegrees(), speed));
  }

  public double getAngleError() {
    Pose2d position = m_tracker.getPose2d();
    double angleInDegrees = position.getRotation().getDegrees();
    double difference = (angleInDegrees - m_targetAngleInDegrees);
    double normDifference = ParadoxField.normalizeAngle(difference);
    return normDifference;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TurnToAngleCommand", 0, "End");
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(getAngleError()) < k_deadZone) || m_cancel.getAsBoolean();
  }
}
