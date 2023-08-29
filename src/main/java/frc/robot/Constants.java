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
  //reach subsystem constants
  public static final int k_reachMotor = 0;
 
  public static class Intake {
    public static final int k_intakeMotor = 7;
  }

 public static class Drive {
    public static final int k_rightDrive = 8; // CAN id
    public static final int k_rightFollower = 9; // CAN id
    public static final int k_leftDrive = 11; // CAN id
    public static final int k_leftFollower = 10; // CAN id
    public static final int k_maxSpeed = 15000;
    public static final double k_ticksPerFeet = 101469.5/9;
    public static final double k_FPSToTPS = 3041/2.7;
    public static final double k_wheelBase = 2.04;
  }

  public static class Arm {
    public static final int k_armMotor = 19; // left CAN id
    public static final int k_armFollower = 4; // right CAN id
    public static final int k_armEncoder = 3; // encoder CAN id
    public static final int k_armBrake = 1; //pneumatic channel
    public static final double k_armTicksToDegrees = 0.0875;
    public static final double k_armZeroAngle = 180.61;
  }

  public static class Wrist {
    public static final int k_wristMotor = 1; //CAN id
    public static final double k_wristTicksToDegrees = 7.65; //3.893;
    public static final double k_wristStartingAngle = 124.48/k_wristTicksToDegrees;
  }

  public static class Camera {
    public static final double k_frontCameraAngle = -3.2;
    public static final double k_rearCameraAngle = 179;
    public static final double k_xFrontCameraOffsetInches = 6.5;
    public static final double k_xRearCameraOffsetInches = -7.5;
  }

  public static class Field {
    public static final double k_startAngleDegrees = -90;
  }
}
