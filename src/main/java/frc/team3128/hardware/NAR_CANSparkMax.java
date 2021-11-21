package frc.team3128.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.team3128.common.NAR_EMotor;

public class NAR_CANSparkMax extends CANSparkMax implements NAR_EMotor{

	private double prevValue = 0;
	private ControlType prevControlMode = ControlType.kVoltage;
	private CANEncoder encoder;
	private SimDevice simMotor;
	private SimDouble simVel;
	private SimDouble simPos;

	/**
	 * 
	 * @param deviceNumber device id
	 * @param type         0 for brushed motor, 1 for brushless motor
	 */
	public NAR_CANSparkMax(int deviceNumber, MotorType type) {
		super(deviceNumber, type);

		encoder = getEncoder();

		simMotor = SimDevice.create("SparkMax",deviceNumber);
		simPos = simMotor.createDouble("Pos", Direction.kBidir, 0);
		simVel = simMotor.createDouble("Vel", Direction.kInput, 0);
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
	public double getSelectedSensorPosition() {
		return encoder.getPosition();
	}

	@Override
	public void setEncoderPosition(double n) {
		encoder.setPosition(n);
	}

	@Override
	public void setRawSimPosition(double pos) {
		simPos.set(pos);
	}

	@Override
	public void setRawSimVelocity(double vel) {
		simVel.set(vel);
	}

	@Override
	public void follow(NAR_EMotor motor) {
		if(!(motor instanceof CANSparkMax)) {
			throw new RuntimeException("bad follow");
		}
		super.follow((CANSparkMax)motor);
	}

	@Override
	public double getSelectedSensorVelocity() {
		return encoder.getVelocity();
	}

	@Override
	public double getMotorOutputVoltage() {
		return getOutputCurrent();
	}

	@Override
	public SpeedController getMotor() {
		return this;
	}
}