// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.HandPosition;
import frc.robot.commands.autos.DriveForTimeCommand;
import frc.robot.commands.autos.DriveUntilPitchCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceAuto extends SequentialCommandGroup {
  /** Creates a new BalanceAuto. */
  private static double k_pitch = 16;
  private static double k_armAngle = 110;
  public BalanceAuto(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, ReachSubsystem reachSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new HandPosition(armSubsystem, reachSubsystem, wristSubsystem, -90, 0, Constants.Wrist.k_wristHomeAngle), 
    new WaitCommand(0.5), 
    new HandPosition(armSubsystem, reachSubsystem, wristSubsystem, k_armAngle, 0, Constants.Wrist.k_wristHomeAngle), 
    new DriveUntilPitchCommand(driveSubsystem, k_pitch), 
    new DriveForTimeCommand(driveSubsystem, 1.75), new AutoBalanceCommand(driveSubsystem));
  }
}
