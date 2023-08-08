// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

//import frc.robot.subsystems.ExampleSubsystem;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Commands;

public final class autos {
  /** Example static factory for an autonomous command. */
  // static CommandBase exampleAuto(ExampleSubsystem subsystem) {
//return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  //}

  private autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command exampleAuto(DriveSubsystem m_driveSubsystem) {
    return null;
  }
}
