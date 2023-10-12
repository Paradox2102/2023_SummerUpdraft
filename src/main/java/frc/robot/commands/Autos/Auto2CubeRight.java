// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.commands.HandPosition2;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2CubeRight extends SequentialCommandGroup {
    /*
     * 1.33, 5.525,90,3,2.4
     * 2.5, 12.5,91.96,3.868,2.797
     * 1.33, 20.808, 90
     */
    private static final Waypoint[] k_out = {
            new Waypoint(1.33, 5.525, Math.toRadians(90)),
            new Waypoint(2.5, 12.5, Math.toRadians(90)),
            new Waypoint(1.33, 20.808, Math.toRadians(90))
    };
    private static Waypoint[] k_back = {
            new Waypoint(1.329, 20.808, Math.toRadians(-90)),
            new Waypoint(2.5, 12.5, Math.toRadians(-90)), 
            new Waypoint(1.33, 6.025, Math.toRadians(-90))
    };

    /** Creates a new TestAuto. */
    public Auto2CubeRight(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, ReachSubsystem reachSubsystem,
            WristSubsystem wristSubsystem, IntakeSubsystem intakeSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelRaceGroup(
                        new IntakeCommand(intakeSubsystem, -0.4),
                        new SequentialCommandGroup(
                                new HandPosition2(armSubsystem, reachSubsystem, wristSubsystem, 102, -102, 0, 18, 15,
                                        () -> false),
                                new CreatePathCommand(driveSubsystem, k_out, true, false, "out"),
                                new HandPosition2(armSubsystem, reachSubsystem, wristSubsystem, 40, -40, 10, -122, 122,
                                        () -> true),
                                new CreatePathCommand(driveSubsystem, k_back, false, true, "back"))),
                new ParallelRaceGroup(
                        new IntakeCommand(intakeSubsystem, 0.3),
                        new WaitCommand(1)),
                new IntakeCommand(intakeSubsystem, 0));
        // new CreatePathCommand(subsystem, k_back, false, true, "back")
    }
}