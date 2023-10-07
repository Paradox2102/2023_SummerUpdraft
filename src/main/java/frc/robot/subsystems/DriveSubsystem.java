// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
// import com.ctre.phoenix.motorcontrol.can.dummy.WPI_TalonFX;
// import com.ctre.phoenix.sensors.dummy.WPI_PigeonIMU;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ApriltagsCamera.ApriltagsCamera;
import frc.ApriltagsCamera.Logger;
import frc.ApriltagsCamera.PositionServer;
import frc.pathfinder.Pathfinder.Path;
import frc.robot.Constants;
import frc.robot.LocationTracker;
import frc.robot.Navigator;
import frc.robot.PositionTracker;
import frc.robot.PositionTrackerPose;
import frc.robot.PurePursuit;
import frc.robot.Sensor;

public class DriveSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_rightDrive = new WPI_TalonFX(Constants.Drive.k_rightDrive);
  private final WPI_TalonFX m_rightFollower = new WPI_TalonFX(Constants.Drive.k_rightFollower);
  private final WPI_TalonFX m_leftDrive = new WPI_TalonFX(Constants.Drive.k_leftDrive);
  private final WPI_TalonFX m_leftFollower = new WPI_TalonFX(Constants.Drive.k_leftFollower);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private static double k_fLeft = (0.05);
  private static double k_fRight = (0.05);
  private static double k_p = 0.1; // 0.025
  private static double k_i = 0.001; //0.0005; //0.00025; //0.000025;
  private static double k_d = 0;
  Navigator m_navigator;
  private Sensor m_sensors;
  private PositionTracker m_posTracker;
  ApriltagsCamera m_frontCamera;
  ApriltagsCamera m_backCamera;
  WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);
  // measures inertia
  LocationTracker m_tracker = new LocationTracker();
  // private final Field2d m_field = new Field2d();
  AprilTagFieldLayout m_aprilTags;
  public PurePursuit m_pursuit;
  Timer m_pathFollowTimer = new Timer();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(ApriltagsCamera frontCamera, ApriltagsCamera backCamera, AprilTagFieldLayout aprilTags) {
    m_pathFollowTimer.reset();
    m_frontCamera = frontCamera;
    m_backCamera = backCamera;
    m_aprilTags = aprilTags;
    m_gyro.reset();

    m_rightDrive.configFactoryDefault();
    m_rightFollower.configFactoryDefault();
    m_leftDrive.configFactoryDefault();
    m_leftFollower.configFactoryDefault();
    m_rightDrive.setInverted(TalonFXInvertType.CounterClockwise);
    m_rightFollower.follow(m_rightDrive);
    m_rightFollower.setInverted(TalonFXInvertType.FollowMaster);
    m_leftDrive.setInverted(TalonFXInvertType.Clockwise);
    m_leftFollower.follow(m_leftDrive);
    m_leftFollower.setInverted(TalonFXInvertType.FollowMaster);
    m_rightDrive.setSelectedSensorPosition(0);
    m_leftDrive.setSelectedSensorPosition(0);
    m_rightDrive.config_kF(0, k_fRight);
    m_rightDrive.config_kP(0, k_p);
    m_rightDrive.config_kI(0, k_i);
    m_rightDrive.config_kD(0, k_d);
    m_leftDrive.config_kF(0, k_fLeft);
    m_leftDrive.config_kP(0, k_p);
    m_leftDrive.config_kI(0, k_i);
    m_leftDrive.config_kD(0, k_d);
    m_sensors = new Sensor(() -> getLeftPosInFeet(),
        () -> getRightPosInFeet(), () -> getLeftSpeedInFPS(),
        () -> getRightSpeedInFPS(), m_gyro);
    m_posTracker = new PositionTrackerPose(0, 0, m_sensors);
    m_navigator = new Navigator(m_posTracker);
    m_navigator.reset(0, 0, 0);
    m_pursuit = new PurePursuit(m_navigator, (l, r) -> setSpeedFPS(l, r), 20);
    m_pursuit.enableLogging("/home/lvuser/logs");
    Logger.log("DriveSubsystem", 0, "DriveSubsystem");
  }

  public void setPower(double leftPower, double rightPower) {
    m_rightDrive.set(TalonFXControlMode.PercentOutput, rightPower);
    m_leftDrive.set(TalonFXControlMode.PercentOutput, leftPower);
    Logger.log("DriveSubsystem", 0,
        String.format("%s, %f, %s, %f", "Set Power Left: ", leftPower, " Right: ", rightPower));
  }

  public void setSpeed(double leftSpeed, double rightSpeed) {
    m_rightDrive.set(TalonFXControlMode.Velocity, rightSpeed);
    m_leftDrive.set(TalonFXControlMode.Velocity, leftSpeed);
  }

  public void setSpeedFPS(double leftSpeed, double rightSpeed) {
    rightSpeed = rightSpeed * Constants.Drive.k_FPSToTPS;
    leftSpeed = leftSpeed * Constants.Drive.k_FPSToTPS;
    // Logger.log("DriveSubsystem", 0, String.format("left speed =%f, right speed =%f", leftSpeed, rightSpeed));
    m_rightDrive.set(TalonFXControlMode.Velocity, rightSpeed);
    m_leftDrive.set(TalonFXControlMode.Velocity, leftSpeed);
  }

  public void stop() {
    setPower(0, 0);
    Logger.log("DriveSubsystem", 0, "Stop");
  }

  public PositionTracker getTracker() {
    return m_posTracker;
  }

  public double getYawInDegrees() {
    return m_posTracker.getPose2d().getRotation().getDegrees();
  }

  public void resetEncoders() {
    m_leftDrive.setSelectedSensorPosition(0);
    m_rightDrive.setSelectedSensorPosition(0);
  }

  // Start autonomous path during Teleop
  public void startPath(Path path, boolean isReversed, boolean setPosition, DoubleSupplier speed, BooleanSupplier cancel) {
    m_pursuit.loadPath(path, isReversed, true, setPosition, speed, cancel);
    m_pursuit.startPath();
    m_pathFollowTimer.reset();
    m_pathFollowTimer.start();
    Logger.log("DriveSubsystem", 0, "Start Path");
  }

  public void endPath() {
    m_pursuit.stopFollow();
    m_pathFollowTimer.stop();
    Logger.log("DriveSubsystem", 0, "End Path");
  }

  public double getRobotY() {
    return m_posTracker.getPose2d().getY();
  }

  public double getRobotX() {
    return m_posTracker.getPose2d().getX();
  }

  public double getLeftPosInFeet() {
    return m_leftDrive.getSelectedSensorPosition() / Constants.Drive.k_ticksPerFeet;
  }

  public double getRightPosInFeet() {
    return m_rightDrive.getSelectedSensorPosition() / Constants.Drive.k_ticksPerFeet;
  }

  public double getLeftPosInTicks() {
    return m_leftDrive.getSelectedSensorPosition();
  }

  public double getRightPosInTicks() {
    return m_rightDrive.getSelectedSensorPosition();
  }

  public double getLeftSpeedInFPS() {
    return m_leftDrive.getSelectedSensorVelocity() / Constants.Drive.k_FPSToTPS;
  }

  public double getRightSpeedInFPS() {
    return m_rightDrive.getSelectedSensorVelocity() / Constants.Drive.k_FPSToTPS;
  }

  public double getLeftSpeedInTicks() {
    return m_leftDrive.getSelectedSensorVelocity();
  }

  public double getRightSpeedInTicks() {
    return m_rightDrive.getSelectedSensorVelocity();
  }

  public double getPitchInDegrees() {
    return m_gyro.getRoll();
  }

  public double computeTargetDegrees(double x0, double y0) {
    Pose2d pose = m_posTracker.getPose2d();
    double y = pose.getY() - y0;
    double x = x0 - pose.getX();
    double m_targetAngle = -Math.atan2(y, x);
    SmartDashboard.putNumber("Target Angle", Math.toDegrees(m_targetAngle));
    SmartDashboard.putNumber("Robot X: ", pose.getX());
    SmartDashboard.putNumber("Robot Y: ", pose.getY());
    return Math.toDegrees(m_targetAngle);
  }

  public OptionalDouble findTargetAngleDegrees() {
    PositionServer.Target target = m_posTracker.m_posServer.getTarget();
    if (target == null) {
      return OptionalDouble.empty();
    }
    return OptionalDouble.of(computeTargetDegrees(target.m_x, target.m_y));
  }

  public boolean isPathFinished() {
    return (m_pursuit.isFinished());
  }

  public void resetGyro(double angle) {
    m_gyro.reset();
  }

  public Sensor getSensors() {
    return m_sensors;
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  public void setBrake(boolean brake) {
    if (brake) {
      m_rightDrive.setNeutralMode(NeutralMode.Brake);
      m_rightFollower.setNeutralMode(NeutralMode.Brake);
      m_leftDrive.setNeutralMode(NeutralMode.Brake);
      m_leftFollower.setNeutralMode(NeutralMode.Brake);
      Logger.log("DriveSubsystem", 0, "Brake Mode");
    } else {
      m_rightDrive.setNeutralMode(NeutralMode.Coast);
      m_rightFollower.setNeutralMode(NeutralMode.Coast);
      m_leftDrive.setNeutralMode(NeutralMode.Coast);
      m_leftFollower.setNeutralMode(NeutralMode.Coast);
    }
  }

  public boolean isFailingChargeStationClimb() {
    // PROBLEM: We should create a getLeftVelocity method that returns FPS.
    // It's not obvious to the reader that this test is checking against 10FPS
    // because the TalonFX velocity reports ticks per 100ms. -Gavin
    return getLeftPosInFeet() < .5 && m_pathFollowTimer.get() > 5;
  }

  public boolean isBalanced() {
    return Math.abs(getPitchInDegrees()) <= 5;
  }

  @Override
  public void periodic() {
    m_drive.feed();
    // SmartDashboard.putNumber("Right Speed", m_rightDrive.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Left Speed", m_leftDrive.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Right Position", m_rightDrive.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Left Position", m_leftDrive.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Right Position Feet", getRightPosInFeet());
    // SmartDashboard.putNumber("Left Position Feet", getLeftPosInFeet());
    // SmartDashboard.putNumber("Gyro Yaw", m_gyro.getAngle());
    // SmartDashboard.putNumber("Gyro Roll", m_gyro.getRoll());
    // SmartDashboard.putNumber("Gyro Pitch", getPitchInDegrees());
    // SmartDashboard.putNumber("Robot Yaw", getYawInDegrees());
    SmartDashboard.putNumber("Robot X",m_posTracker.getPose2d().getX());
    SmartDashboard.putNumber("Robot Y", m_posTracker.getPose2d().getY());
    SmartDashboard.putNumber("Robot angle", m_posTracker.getPose2d().getRotation().getDegrees());
    m_posTracker.update(m_frontCamera, m_backCamera);
    // This method will be called once per scheduler run
  }
}

// arcade drive controls turn by controlling speeds
// tank drive controls with two controllers (one for each side of the robot)
// curviture drive controls doesn't control turn by speeds of wheels, turns with
// perticular curvature
// radius not dependent on speed
// curvitature can not spin in a circle
// raidus and curvature are inverses of each other
// cheesy drive???
