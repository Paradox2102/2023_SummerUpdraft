// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcadeDriveCommand;
//import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.PositionReachCommand;
import frc.robot.commands.ReachCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.MoveWristCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetWristPositionCommand;
import frc.ApriltagsCamera.ApriltagsCamera;
import frc.robot.commands.CalibrateDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // variables for apriltag navigation
  public AprilTagFieldLayout m_aprilTags;
  public final ApriltagsCamera m_frontCamera = new ApriltagsCamera(Constants.Camera.k_xFrontCameraOffsetInches, 0,
      Constants.Camera.k_frontCameraAngle);
  public final ApriltagsCamera m_backCamera = new ApriltagsCamera(Constants.Camera.k_xRearCameraOffsetInches, 0,
      Constants.Camera.k_rearCameraAngle);

  // The robot's subsystems and commands are defined here...
  // The robot's subsystems and commands are defined here...
  final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_frontCamera, m_backCamera, m_aprilTags);
  final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public final ReachSubsystem m_reachSubsystem = new ReachSubsystem(() -> m_armSubsystem.getArmAngleDegrees());
  final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  final WristSubsystem m_wristSubsystem = new WristSubsystem();
  private final CommandJoystick m_stick = new CommandJoystick(0);
  private final CommandJoystick m_stick2 = new CommandJoystick(1);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // TODO: Too many buttons
    m_stick.button(1).whileTrue(new ReachCommand(m_reachSubsystem, 0.4));
    m_stick.button(2).whileTrue(new ReachCommand(m_reachSubsystem, -0.4));
    m_stick.button(3).whileTrue(new PositionReachCommand(m_reachSubsystem, 12));
    m_stick.button(4).whileTrue(new IntakeCommand(m_intakeSubsystem, 0.3));
    m_stick.button(5).whileTrue(new IntakeCommand(m_intakeSubsystem, -0.4));
    m_driveSubsystem.setDefaultCommand(new RunCommand(()// runnable
    -> m_driveSubsystem
        .arcadeDrive(m_stick.getThrottle() > 0 ? -m_stick.getY() : m_stick.getY(), -m_stick.getX()), m_driveSubsystem));
    // runnable : anonymous function takes no argument and returns nothing -> does
    // something called a side effect
    //-> if then statement -> get throttle is the question, ? is the if, : is the else

    m_stick.button(6).whileTrue(new MoveWristCommand(m_wristSubsystem, 0.2));
    m_stick.button(7).whileTrue(new MoveWristCommand(m_wristSubsystem, -0.2));
    m_stick.button(8).onTrue(new SetWristPositionCommand(m_wristSubsystem, 0));
    m_stick.button(9).onTrue(new SetWristPositionCommand(m_wristSubsystem, 90));
    m_stick.button(10).onTrue(new SetWristPositionCommand(m_wristSubsystem, -90));
    m_stick.button(11).whileTrue(new MoveArmCommand(m_armSubsystem, 0.2));
    m_stick.button(12).whileTrue(new MoveArmCommand(m_armSubsystem, -0.2));
    m_stick.button(13).onTrue(new SetArmPositionCommand(m_armSubsystem, 0));
    m_stick.button(14).onTrue(new SetArmPositionCommand(m_armSubsystem, 0));
    // m_stick.button(15).whileTrue(new CalibrateDriveCommand(m_driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

}
