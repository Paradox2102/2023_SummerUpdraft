// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReachCommand;
import frc.robot.commands.ReachPositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  final ReachSubsystem m_reachSubsystem = new ReachSubsystem();

  private final CommandJoystick m_driverController = new CommandJoystick(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_driveSubsystem, ()-> m_driverController.getY(), ()-> m_driverController.getX(), ()-> m_driverController.getThrottle() > 0));
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
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem)); 

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release. 

    // m_driverController.button(1).whileTrue(m_exampleSubsystem.exampleMethodCommand());
      m_driverController.button(1).whileTrue(new IntakeCommand(m_intakeSubsystem, 0.3));
      m_driverController.button(2).whileTrue(new IntakeCommand(m_intakeSubsystem, -0.4));
      m_driverController.button(3).whileTrue(new ReachCommand(m_reachSubsystem, 0.3));
      m_driverController.button(4).whileTrue(new ReachCommand(m_reachSubsystem, -0.3));
      m_driverController.button(5).whileTrue(new ReachPositionCommand(m_reachSubsystem, 27));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

}
