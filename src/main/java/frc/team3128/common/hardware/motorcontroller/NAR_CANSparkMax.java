package frc.team3128.common.hardware.motorcontroller;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

import frc.team3128.common.infrastructure.NAR_EMotor;
import net.thefletcher.revrobotics.CANSparkMax;
import net.thefletcher.revrobotics.SparkMaxRelativeEncoder;
import net.thefletcher.revrobotics.enums.MotorType;

public class NAR_CANSparkMax extends CANSparkMax implements NAR_EMotor {

	private double prevValue = 0;
	private SparkMaxRelativeEncoder encoder;
	private SimDeviceSim encoderSim;
	private SimDouble encoderSimVel;

	/**
	 * 
	 * @param deviceNumber device id
	 * @param type         kBrushed(0) for brushed motor, kBrushless(1) for brushless motor
	 */
	public NAR_CANSparkMax(int deviceNumber, MotorType type) {
		super(deviceNumber, type);

		restoreFactoryDefaults(); // Reset config parameters, unfollow other motor controllers

		encoder = (SparkMaxRelativeEncoder) getEncoder();
		encoder.setPositionConversionFactor(MotorControllerConstants.SPARKMAX_ENCODER_RESOLUTION); // convert rotations to encoder ticks
		encoder.setVelocityConversionFactor(MotorControllerConstants.SPARKMAX_RPM_TO_NUpS); // convert rpm to nu/s

		if(RobotBase.isSimulation()){
			encoderSim = new SimDeviceSim("CANSparkMax[" + this.getDeviceId() + "] - RelativeEncoder");
			encoderSimVel = encoderSim.getDouble("Velocity");
		}
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
	public double getSelectedSensorVelocity() {
		return encoder.getVelocity();
	}

	@Override
	public double getMotorOutputVoltage() {
		return getAppliedOutput() * getBusVoltage();
	}

	@Override
	public void setEncoderPosition(double encPos) {
		encoder.setPosition(encPos);
	}

	@Override
	public void setSimPosition(double pos) {
		encoder.setPosition(pos);
	}

	@Override
	public void setSimVelocity(double vel) {
		encoderSimVel.set(vel);
	}

	@Override
	public void follow(NAR_EMotor motor) {
		if(!(motor instanceof CANSparkMax)) {
			throw new RuntimeException("Bad follow: NAR_CANSparkMax " + getDeviceId() + " attempted to follow non-CANSparkMax motor controller.");
		}
		super.follow((CANSparkMax)motor);
	}
}
