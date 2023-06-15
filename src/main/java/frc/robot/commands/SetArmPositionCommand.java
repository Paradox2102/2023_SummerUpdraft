// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmPositionCommand extends CommandBase {
  ArmSubsystem m_subsystem;
  private double m_setPoint;
  /** Creates a new SetArmPositionCommand. */
  public SetArmPositionCommand(ArmSubsystem subsystem, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_setPoint = setPoint;
    Logger.log("SetArmPositionCommand", 0, "SetArmPositionCommand");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPosition(m_setPoint);
    Logger.log("SetArmPositionCommand", 0, "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("SetArmPositionCommand", 0, "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
