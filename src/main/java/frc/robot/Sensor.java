//copied from normal season code
package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;
// import com.ctre.phoenix.sensors.dummy.PigeonIMU;

public class Sensor implements SensorData{
	private DoubleSupplier m_leftPos;
	private DoubleSupplier m_rightPos;
	private DoubleSupplier m_leftVel;
	private DoubleSupplier m_rightVel;
	private PigeonIMU m_gyro;
	
	public Sensor(DoubleSupplier leftPos, DoubleSupplier rightPos, DoubleSupplier leftVel, DoubleSupplier rightVel, PigeonIMU gyro) {
		m_leftPos = leftPos;
		m_rightPos = rightPos;
		m_leftVel = leftVel;
		m_rightVel = rightVel;
		
		m_gyro = gyro;
	}
	
	public double getLeftEncoderPos() {
		return m_leftPos.getAsDouble();
	}
	
	public double getRightEncoderPos() {
		return m_rightPos.getAsDouble();
	}
	
	public double getLeftEncoderVel() {
		return m_leftVel.getAsDouble();
	}
	
	public double getRightEncoderVel() {
		return m_rightVel.getAsDouble();
	}

	public PigeonIMU getGyro() {
		return m_gyro;
	}
	
	public double getAngle() {
		double[] data = new double[3];
		m_gyro.getYawPitchRoll(data);
		return data[0];
	}
}