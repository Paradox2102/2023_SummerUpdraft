// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {
  private DriveSubsystem m_subsystem;
  private double m_pitchROC;
  private double m_previousPitch;
  private double m_currentPitch;
  private double m_futureAngle;
  private static double k_p = 0.2;
  private double m_power;
  private Timer m_tickTime;
  /** Creates a new AutoBalanceCommand. */
  public AutoBalanceCommand(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_previousPitch = 0;
    m_tickTime.reset();
    m_tickTime.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentPitch = m_subsystem.getPitch();
    m_pitchROC = (m_currentPitch - m_previousPitch)/m_tickTime.get();
    m_tickTime.reset();
    m_futureAngle = m_currentPitch + 0.25 * m_pitchROC;
    m_power = m_futureAngle * k_p;
    m_subsystem.setPower(m_power, m_power);
    m_previousPitch = m_currentPitch;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.isBalanced();
  }
}
