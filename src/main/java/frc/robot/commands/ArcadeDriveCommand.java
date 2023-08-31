// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private final DoubleSupplier m_drive;
  private final DoubleSupplier m_turn;
  private final BooleanSupplier m_reverse;

  /** Creates a new ArcadeDrive. */
  public ArcadeDriveCommand(DriveSubsystem subsystem, DoubleSupplier drive, DoubleSupplier turn,
      BooleanSupplier reverse) {
    // Logger.log("ArcadeDriveCommand", 3, "ArcadeDriveCommand()");

    m_subsystem = subsystem;
    m_drive = drive;
    m_turn = turn;
    m_reverse = reverse;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
    Logger.log("ArcadeDriveCommand", 0, "ArcadeDriveCommand");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ArcadeDriveCommand", 0, "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double drive = m_drive.getAsDouble();
    double turn = m_turn.getAsDouble();

    drive = Math.abs(drive) * drive;
    turn = Math.abs(turn) * turn;

    boolean reverse = m_reverse.getAsBoolean();

    if (!reverse) {
      drive = -drive;
    }

    m_subsystem.setPower(drive + turn, drive - turn);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("ArcadeDriveCommand", 0, "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
