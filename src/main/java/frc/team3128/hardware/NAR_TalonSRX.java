package frc.team3128.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class NAR_TalonSRX extends WPI_TalonSRX{
    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;

	/**
	 * 
	 * @param deviceNumber device id
	 */
	public NAR_TalonSRX(int deviceNumber) {
		super(deviceNumber);
		enableVoltageCompensation(true);
		configVoltageCompSaturation(12, 10);
	}

	@Override
	public void set(ControlMode controlMode, double outputValue) {
		// return;

		if (outputValue != prevValue || controlMode != prevControlMode) {
			super.set(controlMode, outputValue);
			prevValue = outputValue;
		}
	}

	public double getSetpoint() {
		return prevValue;
	}
}