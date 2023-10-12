// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IncArmPositionCommand extends InstantCommand {
  private final ArmSubsystem m_subsystem;
  private final double m_increment;

  public IncArmPositionCommand(ArmSubsystem subsystem, double increment) {
    Logger.log("IncArmPositionCommand", 1, "IncArmPositionCommand()");
    
    m_subsystem = subsystem;
    m_increment = increment;

    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("IncArmPositionCommand", 1, String.format("initialize: inc = %f", m_increment));

    m_subsystem.incrementPosition(m_increment);
  }
}
