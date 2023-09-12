// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

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
  private boolean m_isTipped;
  private boolean m_isBalanced;
  private static double k_p = -0.2;
  private static double k_lookAheadTime = 0.3;
  private static double k_maxSpeed = 0.75;
  private static double k_tippedSpeed = 2.25;
  private static double k_backupTime = 0.175;
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
    m_isBalanced = false;
    m_isTipped = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentPitch = m_subsystem.getPitchInDegrees();
    m_pitchROC = (m_currentPitch - m_previousPitch) / m_tickTime.get();
    if (!m_isTipped) {
      m_tickTime.reset();
      m_futureAngle = m_currentPitch + k_lookAheadTime * m_pitchROC;
      m_power = m_futureAngle * k_p;
      if (Math.abs(m_power) > k_maxSpeed) {
        m_power = k_maxSpeed * Math.signum(m_power);
      }
      if(m_pitchROC > 30) {
        m_isTipped = true;
      }
    } else { 
      m_power = k_tippedSpeed * -Math.signum(m_previousPitch);
      if (m_tickTime.get() > k_backupTime){
        m_isBalanced = true;
      }
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
    return m_isBalanced;
  }
}
