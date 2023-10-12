// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.commands.HandPosition2;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveForwardRight extends SequentialCommandGroup {
  /*
  3.24,5.358, 90
  3.24,12.586,90
  */
  private static final Waypoint[] k_out = {
      new Waypoint(3.24, 5.358, Math.toRadians( 90)),
      new Waypoint(3.24, 12.586, Math.toRadians(90))
  };


  /** Creates a new TestAuto. */
  public AutoDriveForwardRight(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, ReachSubsystem reachSubsystem,
      WristSubsystem wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new HandPosition2(armSubsystem, reachSubsystem, wristSubsystem, 102, -102, 0, 18, 15, () -> false),
        new CreatePathCommand(driveSubsystem, k_out, true, false, "out"));
    // new CreatePathCommand(subsystem, k_back, false, true, "back")
  }
}
