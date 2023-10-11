// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ReachCommand;
import frc.robot.commands.TurnByIncrementCommand;
import frc.robot.commands.TurnToTargetCommand;
import frc.robot.commands.autos.Auto2CubeLeft;
import frc.robot.commands.autos.balance.AutoBalanceCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.MoveWristCommand;
import frc.ApriltagsCamera.ApriltagsCamera;
import frc.robot.commands.AutoPositionArmCommand;
import frc.robot.commands.DriveToTargetCommand;
import frc.robot.commands.HandPosition;
import frc.robot.commands.HandPosition2;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_frontCamera, m_backCamera, m_aprilTags);
  final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public final ReachSubsystem m_reachSubsystem = new ReachSubsystem(() -> m_armSubsystem.getArmAngleDegrees());
  final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  final WristSubsystem m_wristSubsystem = new WristSubsystem(() -> m_armSubsystem.getArmAngleDegrees());
  SendableChooser<Command> m_chooseAuto = new SendableChooser<>();
  
  // We have multiple developers working on different parts of the system, so we
  // set up multiple joysticks
  // All joysticks are available for button use, but only one has control of
  // arcade drive.
  private final CommandJoystick m_driver1 = new CommandJoystick(0);
  final Joystick m_driver1Joystick = new Joystick(0);
  @SuppressWarnings("unused")
  private final CommandJoystick m_driver2 = new CommandJoystick(1);
  private final CommandJoystick m_PRJoystick = new CommandJoystick(2);
  private final CommandJoystick m_BMRJoystick = new CommandJoystick(3);
  private final CommandJoystick m_IAEJoystick = new CommandJoystick(4);
  public final CommandJoystick m_driveStick = m_driver1;
  // private final CommandJoystick m_stick2 = new CommandJoystick(1);

  private final double k_cancelThreshold = 0.50; // Joystick x threshold for canceling commands

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    m_armSubsystem.setExtent(() -> m_reachSubsystem.getDistance());   // Gives the arm subsystem access to the current arm extent
    m_frontCamera.connect("10.21.2.10", 5800);
    m_backCamera.connect("10.21.2.11", 5800);
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

    m_driveSubsystem.setDefaultCommand(new RunCommand(()// runnable
    -> m_driveSubsystem
        .arcadeDrive(m_driveStick.getThrottle() > 0 ? -m_driveStick.getY() : m_driveStick.getY(), -m_driveStick.getX()),
        m_driveSubsystem));

    // runnable : anonymous function takes no argument and returns nothing -> does
    // something called a side effect
    // -> if then statement -> get throttle is the question, ? is the if, : is the
    // else

    configureBindingsPR();
    configureBindingsBMR();
    configureBindingsIAE();
  }

  private void configureBindingsPR() {

    // Paul's test joystick
    // calibrate ticks to feet conversion
    m_PRJoystick.button(1).onTrue(new HandPosition(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 0, 0, 0));
    m_PRJoystick.button(2).onTrue(new AutoBalanceCommand(m_driveSubsystem));
    m_PRJoystick.button(3).whileTrue(new MoveArmCommand(m_armSubsystem, 0.25));
    m_PRJoystick.button(4).whileTrue(new MoveArmCommand(m_armSubsystem, -0.25));
    m_PRJoystick.button(5).whileTrue(new MoveWristCommand(m_wristSubsystem, 0.25));
    m_PRJoystick.button(6).whileTrue(new MoveWristCommand(m_wristSubsystem, -0.25));
    m_PRJoystick.button(7)
        .onTrue(new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 40, -40, 10, -122, 122, () -> true));
    m_PRJoystick.button(9).onTrue(
        new HandPosition(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 0, 0, Constants.Wrist.k_wristHomeAngle));
    m_PRJoystick.button(10).onTrue(new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 103, -103, 0,
        18, 15, () -> m_PRJoystick.getThrottle() < 0));
    // Outtake
    m_PRJoystick.button(11).whileTrue(new IntakeCommand(m_intakeSubsystem, 0.3));

    // Intake toggle
    m_PRJoystick.button(12).toggleOnTrue(new IntakeCommand(m_intakeSubsystem, -0.4));
  }

  private void configureBindingsBMR() {
    // Briselda's test joystick
    // lowest reach, straight up
    m_BMRJoystick.button(3).onTrue(new HandPosition(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 0, 0, 122));
    // CONE low position
    // m_BMRJoystick.button(4).onTrue(new HandPosition2(m_armSubsystem,
    // m_reachSubsystem, m_wristSubsystem, 0, 2, -126, ()->
    // m_driveStick.getThrottle() < 0));
    // CONE middle position
    // m_BMRJoystick.button(5).onTrue(new HandPosition2(m_armSubsystem,
    // m_reachSubsystem, m_wristSubsystem, 35, 1, -50, ()->
    // m_driveStick.getThrottle() < 0));
    // CONE top position
    // m_BMRJoystick.button(6).onTrue(new HandPosition2(m_armSubsystem,
    // m_reachSubsystem, m_wristSubsystem, 42, 24, -52, ()->
    // m_driveStick.getThrottle() < 0));
    // CUBE low position
    m_BMRJoystick.button(4)
        .onTrue(new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, -55, 0, 20, 0, 0,
            () -> m_driveStick.getThrottle() < 0));
    // CUBE middle position
    m_BMRJoystick.button(5)
        .onTrue(new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, -60, 0, 0, 0, 0,
            () -> m_driveStick.getThrottle() < 0));
    // CUBE top position
    m_BMRJoystick.button(6)
        .onTrue(new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, -50, 16, 0, 0, 0,
            () -> m_driveStick.getThrottle() < 0));

    m_BMRJoystick.button(7).whileTrue(new MoveArmCommand(m_armSubsystem, 0.2));
    m_BMRJoystick.button(8).whileTrue(new MoveArmCommand(m_armSubsystem, -0.2));

    m_BMRJoystick.button(9).whileTrue(new ReachCommand(m_reachSubsystem, 0.4));
    m_BMRJoystick.button(10).whileTrue(new ReachCommand(m_reachSubsystem, -0.4));

    m_BMRJoystick.button(11).whileTrue(new MoveWristCommand(m_wristSubsystem, 0.2));
    m_BMRJoystick.button(12).whileTrue(new MoveWristCommand(m_wristSubsystem, -0.2));
    // outtake
    m_BMRJoystick.button(1).whileTrue(new IntakeCommand(m_intakeSubsystem, 0.3));
    // intake
    m_BMRJoystick.button(2).toggleOnTrue(new IntakeCommand(m_intakeSubsystem, -0.4));

    // m_stick.button(15).whileTrue(new CalibrateDriveCommand(m_driveSubsystem));

  }

  Boolean cancelCommand() {
    // return Math.abs(m_driveStick.getX()) > k_cancelThreshold;
    return m_driver1Joystick.getRawButton(8);
  }

  Boolean isReversed() {
    PositionTracker tracker = m_driveSubsystem.getTracker();
    return tracker.m_posServer.getTarget().isPathReversed(tracker);
  }

  private void configureBindingsIAE() {
    // Driver 1 joystick

    // Move arm to vertical
    m_driver1.button(3).onTrue(
        new HandPosition(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 0, 0, Constants.Wrist.k_wristHomeAngle));

    // Outtake
    m_driver1.button(1).whileTrue(new IntakeCommand(m_intakeSubsystem, 0.3));

    // Intake toggle
    m_driver1.button(2).toggleOnTrue(new IntakeCommand(m_intakeSubsystem, -0.4));

    // Auto position arm
    m_driver1.button(5).onTrue(new AutoPositionArmCommand(m_driveSubsystem, () -> isReversed(),
        m_armSubsystem, m_reachSubsystem, m_wristSubsystem));

    // Drive to target
    m_driver1.button(4).onTrue(new TurnToTargetCommand(m_driveSubsystem, m_armSubsystem,
        m_reachSubsystem, m_wristSubsystem, () -> cancelCommand(), false, ()->m_driveStick.getY()));

    // Drive To Target
    m_driver1.button(6).onTrue(new DriveToTargetCommand(m_driveSubsystem, m_armSubsystem, m_reachSubsystem,
        m_wristSubsystem, () -> m_driver1.getY(), () -> cancelCommand()));

    m_driver1.button(7).onTrue(new HandPosition2(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 102, -102, 0, 18, 15,() -> m_driveStick.getThrottle() < 0));


    m_driver1.povRight().onTrue(new TurnByIncrementCommand(m_driveSubsystem, 1.5, () -> cancelCommand()));
    m_driver1.povLeft().onTrue(new TurnByIncrementCommand(m_driveSubsystem, -1.5, () -> cancelCommand()));

    // Driver 2 joystick

    // None assigned

    // Isa's test joystick
    m_IAEJoystick.button(1).onTrue(
        new HandPosition(m_armSubsystem, m_reachSubsystem, m_wristSubsystem, 0, 0, Constants.Wrist.k_wristHomeAngle));

    // m_IAEJoystick.button(8).onTrue(new SetWristPositionCommand(m_wristSubsystem,
    // -90));

    // m_IAEJoystick.button(8).onTrue(new SetWristPositionCommand(m_wristSubsystem,
    // -90));

    m_IAEJoystick.button(5).whileTrue(new MoveArmCommand(m_armSubsystem, 0.2));
    m_IAEJoystick.button(3).whileTrue(new MoveArmCommand(m_armSubsystem, -0.2));

    m_IAEJoystick.button(6).whileTrue(new ReachCommand(m_reachSubsystem, 0.4));
    m_IAEJoystick.button(4).whileTrue(new ReachCommand(m_reachSubsystem, -0.4));

    m_IAEJoystick.button(11).whileTrue(new MoveWristCommand(m_wristSubsystem, 0.2));
    m_IAEJoystick.button(12).whileTrue(new MoveWristCommand(m_wristSubsystem, -0.2));

    m_IAEJoystick.button(9).whileTrue(new IntakeCommand(m_intakeSubsystem, 0.3));
    m_IAEJoystick.button(10).toggleOnTrue(new IntakeCommand(m_intakeSubsystem, -0.4));

    // Angle Turn Rotation
    m_IAEJoystick.button(2).onTrue(new AutoPositionArmCommand(m_driveSubsystem, () -> m_driveStick.getThrottle() < 0,
        m_armSubsystem, m_reachSubsystem, m_wristSubsystem));
    // m_IAEJoystick.button(2).onTrue(new TurnToAngleCommand(m_driveSubsystem, 0, ()
    // -> cancelCommand()));
    m_IAEJoystick.button(7).onTrue(new TurnToTargetCommand(m_driveSubsystem, m_armSubsystem,
        m_reachSubsystem, m_wristSubsystem, () -> cancelCommand(), false, ()->m_driveStick.getY()));

    // Drive To Target
    m_IAEJoystick.button(8).onTrue(new DriveToTargetCommand(m_driveSubsystem, m_armSubsystem, m_reachSubsystem,
        m_wristSubsystem, /*() -> m_driveStick.getThrottle() < 0,*/ () -> m_IAEJoystick.getY(), () -> cancelCommand()));

    m_IAEJoystick.povRight().onTrue(new TurnByIncrementCommand(m_driveSubsystem, -1.5, () -> cancelCommand()));
    m_IAEJoystick.povLeft().onTrue(new TurnByIncrementCommand(m_driveSubsystem, 1.5, () -> cancelCommand()));

    // auto position command
    // m_IAEJoystick.button(2).onTrue(new AutoPositionArmCommand(m_driveSubsystem,
    // () -> m_driveStick.getThrottle() < 0, m_armSubsystem, m_reachSubsystem,
    // m_wristSubsystem));

    // bottom cone position
    // m_IAEJoystick.button(2).onTrue(new HandPosition2(m_armSubsystem,
    // m_reachSubsystem, m_wristSubsystem, 42, -42, 1, -95, 85,
    // () -> m_driveStick.getThrottle() < 0));

    // middle cone position
    // m_IAEJoystick.button(2).onTrue(new HandPosition2(m_armSubsystem,
    // m_reachSubsystem, m_wristSubsystem, 35, -35, 9, -95, 95,
    // () -> m_driveStick.getThrottle() < 0));

    // top cone position
    // m_IAEJoystick.button(7).onTrue(new HandPosition2(m_armSubsystem,
    // m_reachSubsystem, m_wristSubsystem, 42, -42, Constants.Reach.k_maxReach, -95,
    // 85,
    // () -> m_driveStick.getThrottle() < 0));

    // bottom cube position
    // m_IAEJoystick.button(2).onTrue(new HandPosition2(m_armSubsystem,
    // m_reachSubsystem, m_wristSubsystem, 103, -96, 0, -18, 15,
    // () -> m_driveStick.getThrottle() < 0));

    // middle cube position
    // m_IAEJoystick.button(2).onTrue(new HandPosition2(m_armSubsystem,
    // m_reachSubsystem, m_wristSubsystem, 40, -40, 10, -122, 122,
    // () -> m_driveStick.getThrottle() < 0));

    // top cube position
    // m_IAEJoystick.button(7).onTrue(new HandPosition2(m_armSubsystem,
    // m_reachSubsystem, m_wristSubsystem, 47, -47, 27, -109, 109,
    // () -> m_driveStick.getThrottle() < 0));

    // feeder cone position
    // m_IAEJoystick.button(7).onTrue(new HandPosition2(m_armSubsystem,
    // m_reachSubsystem, m_wristSubsystem, 47, -47, 0, -24, 24,
    // () -> m_driveStick.getThrottle() < 0));

    // feeder cube position
    // m_IAEJoystick.button(7).onTrue(new HandPosition2(m_armSubsystem,
    // m_reachSubsystem, m_wristSubsystem, 22, -22, 9, -145, 145,
    // () -> m_driveStick.getThrottle() < 0));
    // 0 extent -24 wrist 57 arm
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return m_chooseAuto.getSelected();
    // return new BalanceAuto(m_driveSubsystem, m_armSubsystem, m_wristSubsystem,
    // m_reachSubsystem);
    return new Auto2CubeLeft(m_driveSubsystem, m_armSubsystem, m_reachSubsystem, m_wristSubsystem, m_intakeSubsystem);
  }
}
