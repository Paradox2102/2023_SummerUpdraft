//copied from normal season code
package frc.robot;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.ApriltagsCamera.ApriltagLocation;
import frc.ApriltagsCamera.ApriltagsCamera;
import frc.ApriltagsCamera.Logger;
import frc.ApriltagsCamera.PositionServer;

// import frc.lib.CSVWriter;
// import frc.lib.CSVWriter.Field;

public class PositionTrackerPose extends PositionTracker {
	private SensorData m_sensors;
	private DifferentialDrivePoseEstimator m_poseEstimator;
	// public PositionServer m_posServer;

	public PositionTrackerPose(double x, double y, SensorData sensor) {
		super();

		m_sensors = sensor;
		// For the extended constructor, the default values are:
		// VecBuilder.fill(0.02, 0.02, 0.01) - SD of internal state
		// VecBuilder.fill(0.1, 0.1, 0.1)) - SD of vision pose measurment
		// Based on the experiments performed on 2023-03-17 Friday, the vison pose estimates (in metres, metres, radians) should be between:
		// env vs cam: 0.038299922	0.03770631627	0.01169028658
		// est vs cam: 0.01801886372	0.01776299463	0.007439846418
		// As these are significantly lower than the defaults, using them would make us trust the camera more.
		// Slicing the data by distance, I see a stong linear relationship between distance and SD for all of X, Y, and angle,
		// but at the maximum distance the errors are still less than the default.
		// -Gavin
		m_poseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(Constants.Drive.k_wheelBase),
			ParadoxField.rotation2dFromParadoxAngle(Constants.Field.k_startAngleDegrees), 
			0, 0, ParadoxField.pose2dFromParadox(0, 0, Constants.Field.k_startAngleDegrees));
	}
		
	public PositionTracker.PositionContainer getPos() {
		Pose2d pose = ParadoxField.pose2dFromFRC(m_poseEstimator.getEstimatedPosition());
		return new PositionTracker.PositionContainer(pose.getX(), pose.getY());
	}

	@Override
	public Pose2d getPose2d() {
		return ParadoxField.pose2dFromFRC(m_poseEstimator.getEstimatedPosition());
	}

	public void setXYAngle(double x, double y, double angleInDegrees) {
		Logger.log("PositionTracker", 1, String.format("x=%f, y=%f, angle=%f", x, y, angleInDegrees));
		m_sensors.getGyro().setYaw(angleInDegrees);
		m_poseEstimator.resetPosition(
			ParadoxField.rotation2dFromParadoxAngle(angleInDegrees), 
			ParadoxField.distanceFromParadox(m_sensors.getLeftEncoderPos()), 
			ParadoxField.distanceFromParadox(m_sensors.getRightEncoderPos()), 
			ParadoxField.pose2dFromParadox(x, y, angleInDegrees)
		);
	}

	public void setAngle(double angle) {
		m_sensors.getGyro().setYaw(angle);
	}

	@Override
	public double getLeftEncoderPos() {
		return m_sensors.getLeftEncoderPos();
	}

	@Override
	public double getRightEncoderPos() {
		return m_sensors.getRightEncoderPos();
	}

	@Override
	public double getLeftEncoderVel() {
		return m_sensors.getLeftEncoderVel();
	}

	@Override
	public double getRightEncoderVel() {
		return m_sensors.getRightEncoderVel();
	}

	public double getGyroAngleDegrees() {
		return m_sensors.getAngle();
	}

	public static class PositionContainer {
		public double x, y;

		public PositionContainer(double x, double y) {
			this.x = x;
			this.y = y;
		}
	}

	ApriltagLocation tags[] = { new ApriltagLocation(1, 0, 0, -90) }; 
	
	@Override
	public void update(ApriltagsCamera frontCamera, ApriltagsCamera rearCamera){

		m_poseEstimator.updateWithTime(ApriltagsCamera.getTime(), ParadoxField.rotation2dFromParadoxAngle(getGyroAngleDegrees()), 
										ParadoxField.distanceFromParadox(getLeftEncoderPos()), 
										ParadoxField.distanceFromParadox(getRightEncoderPos()));
		
		// if (!DriverStation.isAutonomous()) {
			frontCamera.processRegions(m_poseEstimator);
			rearCamera.processRegions(m_poseEstimator);
		// }

		m_posServer.setAllianceColor(DriverStation.getAlliance() == DriverStation.Alliance.Red);
		SmartDashboard.putBoolean("Red Alliance", DriverStation.getAlliance() == DriverStation.Alliance.Red);
		Pose2d pos = m_poseEstimator.getEstimatedPosition();
		pos = ParadoxField.pose2dFromFRC(pos);
		SmartDashboard.putNumber("xPos", pos.getX());
		SmartDashboard.putNumber("yPos", pos.getY());
		SmartDashboard.putNumber("Robot Angle", pos.getRotation().getDegrees());
		m_posServer.setPosition(pos.getX(), pos.getY(), pos.getRotation().getDegrees());

		PositionServer.Target target = m_posServer.getTarget();

		if (target != null) {
		//   Logger.log("PositionTracker", 1, String.format("x=%f,y=%f,h=%f", target.m_x, target.m_y, target.m_h));
		  SmartDashboard.putNumber("TargetX", target.m_x);
		  SmartDashboard.putNumber("TargetY", target.m_y);
		  SmartDashboard.putNumber("TargetH", target.m_h);
		}
	} 
}