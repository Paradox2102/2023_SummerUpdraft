// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.ParadoxField;
import frc.robot.PositionTracker;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private final double m_targetAngleInDegrees;
  private final PositionTracker m_tracker;
  private static final double k_minPower = 0.07;
  private static final double k_deadZone = 1;


  /** Creates a new TurnToAngleCommand. */
  public TurnToAngleCommand(DriveSubsystem subsystem, double targetAngleInDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_targetAngleInDegrees = targetAngleInDegrees;
    m_tracker = m_subsystem.getTracker();
    Logger.log("TurnToAngleCommand", 0, "TurnToAngleCommand");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TurnToAngleCommand", 0, "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d position = m_tracker.getPose2d();
    double normDifference = getAngleError();
    double power = Constants.Camera.k_p * normDifference;
    if (Math.abs(power) < k_minPower) {
      power = k_minPower * Math.signum(power);
    }
    m_subsystem.setPower(power, -power);
    SmartDashboard.putNumber("target angle", m_targetAngleInDegrees);
    SmartDashboard.putNumber("current", normDifference);
    SmartDashboard.putNumber("turnPower", power);
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(getAngleError()) < k_deadZone);
  }
}
