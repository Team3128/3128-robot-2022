package frc.team3128.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class AN_TalonFX extends WPI_TalonFX{
	private boolean lazy;
    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;

	/**
	 * 
	 * @param deviceNumber device id
	 * @param isLazy whether or not the motor is lazy
	 */
	public AN_TalonFX(int deviceNumber, boolean isLazy) {
		super(deviceNumber);
		enableVoltageCompensation(true);
		configVoltageCompSaturation(12, 10);
		lazy = isLazy;
	}

	/**
	 * 
	 * @param deviceNumber device id
	 */
	public AN_TalonFX(int deviceNumber){
		this(deviceNumber, true);
	}

	public void setLazy(boolean isLazy){
		lazy = isLazy;
	}

	public boolean isLazy(){
		return lazy;
	}

	@Override
	public void set(ControlMode controlMode, double outputValue) {
		if(lazy){
			if (outputValue != prevValue || controlMode != prevControlMode) {
				super.set(controlMode, outputValue);
				prevValue = outputValue;
			}
		}else{
			super.set(controlMode, outputValue);
		}
	}

	public double getSetpoint() {
		return prevValue;
	}
}