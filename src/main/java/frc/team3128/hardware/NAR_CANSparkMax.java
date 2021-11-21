package frc.team3128.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.team3128.common.NAR_EMotor;

public class NAR_CANSparkMax extends CANSparkMax implements NAR_EMotor{

	private double prevValue = 0;
	private ControlType prevControlMode = ControlType.kVoltage;
	private CANEncoder encoder;
	private EncoderSim encoderSim;

	/**
	 * 
	 * @param deviceNumber device id
	 * @param type         0 for brushed motor, 1 for brushless motor
	 */

	public NAR_CANSparkMax(int deviceNumber, MotorType type) {
		super(deviceNumber, type);
		encoder = getEncoder();
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

	@Override
	public double getEncoderPosition() {
		return encoder.getPosition();
	}

	@Override
	public void setEncoderPosition(double n) {
		encoder.setPosition(n);
	}

	@Override
	public void setRawSimPosition(double pos) {
		encoderSim.setDistance(pos);
	}

	@Override
	public void setRawSimVelocity(double vel) {
		encoderSim.setRate(vel);
	}

	@Override
	public void follow(NAR_EMotor motor) {
		if(!(motor instanceof CANSparkMax)) {
			throw new RuntimeException("bad follow");
		}
		super.follow((CANSparkMax)motor);
	}
}