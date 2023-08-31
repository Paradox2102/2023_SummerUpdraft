// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto3 extends SequentialCommandGroup {
  private static Waypoint[] k_out1 = {
    new Waypoint(1.371,5.525, 90),
    new Waypoint(1.828, 21.808, 90)
  };
  private static Waypoint[] k_back1 = {
    new Waypoint(1.828, 21.808, -90),
    new Waypoint (3.15, 5.525,-90)
  };
  private static Waypoint[] k_out2 = {
    new Waypoint(3.15, 5.525,90,11.465,2.743),
    new Waypoint(-2.326, 21.808, 90)
  };
  private static Waypoint[] k_back2 = {
    new Waypoint(-2.326, 21.808, -90),
    new Waypoint(3.822,14.912,-90),
    new Waypoint(-0.665,5.525,-90)
  };
  /** Creates a new TestAuto2. */
  public TestAuto3(DriveSubsystem subsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new CreatePathCommand(subsystem, k_out1, true, false, "out"), new CreatePathCommand(subsystem, k_back1, false, true, "back"), new CreatePathCommand(subsystem, k_out2, false, false, "out 2"), new CreatePathCommand(subsystem, k_back2, false, true, "back 2"));
  }
}
