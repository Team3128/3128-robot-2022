package frc.team3128.common.hardware.motorcontroller;

import com.revrobotics.REVLibError;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.team3128.common.hardware.motor.NAR_Motor;
import net.thefletcher.revrobotics.CANSparkMax;
import net.thefletcher.revrobotics.SparkMaxRelativeEncoder;
import net.thefletcher.revrobotics.enums.MotorType;

public class NAR_CANSparkMax extends CANSparkMax {

	private double prevValue = 0;
	private SparkMaxRelativeEncoder encoder;
	private SimDeviceSim encoderSim;
	private SimDouble encoderSimVel;
	private NAR_Motor motor;

	/**
	 * 
	 * @param deviceNumber device id
	 * @param type         kBrushed(0) for brushed motor, kBrushless(1) for brushless motor
	 */
	 public NAR_CANSparkMax(int deviceNumber, MotorType type, NAR_Motor motor) {
		super(deviceNumber, type);
		this.motor = motor;

		restoreFactoryDefaults(); // Reset config parameters, unfollow other motor controllers

		encoder = (SparkMaxRelativeEncoder) getEncoder();
		encoder.setPositionConversionFactor(MotorControllerConstants.SPARKMAX_ENCODER_RESOLUTION); // convert rotations to encoder ticks
		encoder.setVelocityConversionFactor(MotorControllerConstants.SPARKMAX_RPM_TO_NUpS); // convert rpm to nu/s

		if(RobotBase.isSimulation()){
			encoderSim = new SimDeviceSim("CANSparkMax[" + this.getDeviceId() + "] - RelativeEncoder");
			encoderSimVel = encoderSim.getDouble("Velocity");
		}
	}

	public NAR_CANSparkMax(int deviceNumber, MotorType type) {
		this(deviceNumber, type, null);
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

	public double getSelectedSensorPosition() {
		return encoder.getPosition();
	}

	public double getSelectedSensorVelocity() {
		return encoder.getVelocity();
	}

	public double getMotorOutputVoltage() {
		return getAppliedOutput() * getBusVoltage();
	}

	public void setEncoderPosition(double encPos) {
		encoder.setPosition(encPos);
	}

	public void setSimPosition(double pos) {
		encoder.setPosition(pos);
	}

	public void setSimVelocity(double vel) {
		encoderSimVel.set(vel);
	}

	@Override
	public REVLibError follow(CANSparkMax motor) {
		return super.follow((CANSparkMax)motor);
	}

	// public void follow(NAR_EMotor motor) {
	// 	if(!(motor instanceof CANSparkMax)) {
	// 		throw new RuntimeException("Bad follow: NAR_CANSparkMax " + getDeviceId() + " attempted to follow non-CANSparkMax motor controller.");
	// 	}
	// 	super.follow((CANSparkMax)motor);
	// }
}
