// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autos.Autos;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.MoveWristCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetWristPositionCommand;
import frc.ApriltagsCamera.ApriltagsCamera;
import frc.robot.commands.CalibrateDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //variables for apriltag navigation
  public AprilTagFieldLayout m_aprilTags;
  public final ApriltagsCamera m_frontCamera = new ApriltagsCamera(Constants.Camera.k_xFrontCameraOffsetInches, 0, Constants.Camera.k_frontCameraAngle);
  public final ApriltagsCamera m_backCamera = new ApriltagsCamera(Constants.Camera.k_xRearCameraOffsetInches, 0, Constants.Camera.k_rearCameraAngle);

  // The robot's subsystems and commands are defined here...
  final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_frontCamera, m_backCamera, m_aprilTags);
  final WristSubsystem m_wristSubsystem = new WristSubsystem();
  final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final CommandJoystick m_stick = new CommandJoystick(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));
      m_driveSubsystem.setDefaultCommand(new RunCommand(()->m_driveSubsystem.arcadeDrive(m_stick.getThrottle() > 0 ? -m_stick.getY(): m_stick.getY(), -m_stick.getX()), m_driveSubsystem));
    m_stick.button(1).whileTrue(new MoveWristCommand(m_wristSubsystem, 0.2));
    m_stick.button(2).whileTrue(new MoveWristCommand(m_wristSubsystem, -0.2));
    m_stick.button(3).onTrue(new SetWristPositionCommand(m_wristSubsystem, 0));
    m_stick.button(4).onTrue(new SetWristPositionCommand(m_wristSubsystem, 90));
    m_stick.button(5).onTrue(new SetWristPositionCommand(m_wristSubsystem, -90));
    m_stick.button(6).whileTrue(new MoveArmCommand(m_armSubsystem, 0.2));
    m_stick.button(7).whileTrue(new MoveArmCommand(m_armSubsystem, -0.2));
    m_stick.button(8).onTrue(new SetArmPositionCommand(m_armSubsystem, 0));
    m_stick.button(9).onTrue(new SetArmPositionCommand(m_armSubsystem, 0));
    m_stick.button(10).whileTrue(new CalibrateDriveCommand(m_driveSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_driveSubsystem);
  }
}
