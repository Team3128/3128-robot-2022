package frc.team3128.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class GoodTalonFX extends WPI_TalonFX{
    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;

	/**
	 * 
	 * @param deviceNumber device id
	 */
	public GoodTalonFX(int deviceNumber) {
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