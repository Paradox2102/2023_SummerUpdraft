//copied from normal season code
package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
// import com.ctre.phoenix.sensors.dummy.PigeonIMU;

public interface SensorData {
	double getLeftEncoderPos(); //must return in feet
	double getRightEncoderPos(); //must return in feet
	double getLeftEncoderVel(); //must return in feet/sec
	double getRightEncoderVel(); //must return in feet/sec
	double getAngle();
	PigeonIMU getGyro();
}