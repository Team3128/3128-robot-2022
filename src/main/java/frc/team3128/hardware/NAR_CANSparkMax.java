package frc.team3128.hardware;

import com.revrobotics.CANSparkMax;

public class NAR_CANSparkMax extends CANSparkMax {

	private double prevValue = 0;
	/**
	 * 
	 * @param deviceNumber device id
	 * @param type         0 for brushed motor, 1 for brushless motor
	 */

	public NAR_CANSparkMax(int deviceNumber, MotorType type) {
		super(deviceNumber, type);
		// enableVoltageCompensation(true);
		// configVoltageCompSaturation(12, 10);
	}

	@Override
	public void set(double outputValue) {

		if (outputValue != prevValue) {
			super.set(outputValue);
			prevValue = outputValue;
		}

	}

	public double getSetpoint() {
		return prevValue;
	}
}
