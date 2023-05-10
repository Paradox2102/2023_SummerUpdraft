// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Drive {
    public static final int k_rightDrive = 8; // CAN id
    public static final int k_rightFollower = 9; // CAN id
    public static final int k_leftDrive = 11; // CAN id
    public static final int k_leftFollower = 10; // CAN i
  }

  public static class Wrist {
    public static final int k_wristMotor = 1; //CAN id
    public static final double k_wristTicksToDegrees = 3.893;
    public static final double k_wristStartingAngle = 124.48/k_wristTicksToDegrees;
  }
}
