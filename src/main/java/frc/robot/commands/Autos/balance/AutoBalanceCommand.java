// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.balance;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {
  private DriveSubsystem m_subsystem;
  private double m_pitchROC;
  private double m_previousPitch;
  private double m_currentPitch;
  private double m_futureAngle;
  private static double k_p = -0.2;
  private static double k_lookAheadTime = 0.3;
  private static double k_maxSpeed = 0.5;
  private double m_power;
  private Timer m_tickTime = new Timer();

  /** Creates a new AutoBalanceCommand. */
  public AutoBalanceCommand(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
    Logger.log("AutoBalanceCommand", 0, "AutoBalanceCommand");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("AutoBalanceCommand", 0, "Initialize");
    m_previousPitch = m_subsystem.getPitchInDegrees();
    m_tickTime.reset();
    m_tickTime.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentPitch = m_subsystem.getPitchInDegrees();
    m_pitchROC = (m_currentPitch - m_previousPitch) / m_tickTime.get();
    m_tickTime.reset();
    m_futureAngle = m_currentPitch + k_lookAheadTime * m_pitchROC;
    m_power = m_futureAngle * k_p;
    if (Math.abs(m_power) > k_maxSpeed) {
      m_power = k_maxSpeed * Math.signum(m_power);
    }
    SmartDashboard.putNumber("Charge Station Power", m_power);
    SmartDashboard.putNumber("Pitch Rate Of Change", m_pitchROC);
    m_subsystem.setSpeedFPS(m_power, m_power);
    m_previousPitch = m_currentPitch;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("AutoBalanceCommand", 0, "End");
    m_subsystem.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.isBalanced() && Math.abs(m_pitchROC) <= 1; 
  }
}
