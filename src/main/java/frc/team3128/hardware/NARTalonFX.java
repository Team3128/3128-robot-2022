package frc.team3128.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.team3128.common.NARMotor;
import frc.team3128.common.NARTalon;

public class NARTalonFX extends WPI_TalonFX{
	private boolean lazy;
    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;

	/**
	 * 
	 * @param deviceNumber device id
	 * @param isLazy whether or not the motor is lazy
	 */
	public NARTalonFX(int deviceNumber, boolean isLazy) {
		super(deviceNumber);
		enableVoltageCompensation(true);
		configVoltageCompSaturation(12, 10);
		lazy = isLazy;
	}

	/**
	 * 
	 * @param deviceNumber device id
	 */
	public NARTalonFX(int deviceNumber){
		this(deviceNumber, true);
	}

	@Override
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

	@Override
	public double getSetpoint() {
		return prevValue;
	}
}