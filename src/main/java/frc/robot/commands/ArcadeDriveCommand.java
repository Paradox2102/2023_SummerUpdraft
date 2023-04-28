// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//SAVED BUT NOT COMMITED

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  /** Creates a new ArcadeDriveCommand. */
  DriveSubsystem m_subsystem;
  DoubleSupplier m_drive;
  DoubleSupplier m_turn;
  

  public ArcadeDriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier drive, DoubleSupplier turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = driveSubsystem; 
    m_drive = drive;
    m_turn = turn;

    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drive = m_drive.getAsDouble();
    double turn = m_turn.getAsDouble();

    drive = drive * drive * Math.signum(drive);
    turn = turn * turn * Math.signum(turn);

    m_subsystem.setPower(drive + turn, drive - turn);
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
