package frc.team3128.common.hardware.motor;

// import com.revrobotics.ControlType;
// import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;

import frc.team3128.Robot;
import frc.team3128.common.NAR_EMotor;

import net.thefletcher.revrobotics.CANEncoder;
import net.thefletcher.revrobotics.CANSparkMax;
import net.thefletcher.revrobotics.enums.MotorType;

public class NAR_CANSparkMax extends CANSparkMax implements NAR_EMotor{

	private double prevValue = 0;
	private CANEncoder encoder;
	private SimDevice encoderSim;
	private SimDouble encoderPos;
	private SimDouble encoderVel;

	/**
	 * 
	 * @param deviceNumber device id
	 * @param type         0 for brushed motor, 1 for brushless motor
	 */
	public NAR_CANSparkMax(int deviceNumber, MotorType type) {
		super(deviceNumber, type);

		if(Robot.isReal()){
			encoder = getEncoder();
		}else{
			encoderSim = SimDevice.create("CANEncoder", deviceNumber);
			encoderPos = encoderSim.createDouble("Pos", Direction.kBidir, 0);
			encoderVel = encoderSim.createDouble("Vel", Direction.kBidir, 0);
		}

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
		if(encoderSim != null)
			return encoderPos.get();	
		else
			return encoder.getPosition();
	}

	@Override
	public double getSelectedSensorVelocity() {
		if(encoderSim != null)
			return encoderVel.get();	
		else
			return encoder.getVelocity();
	}

	@Override
	public double getMotorOutputVoltage() {
		return getAppliedOutput();
	}

	@Override
	public void setEncoderPosition(double n) {
		if(encoderSim != null)
			encoderPos.set(n);	
		else
			setEncPosition(n);
	}

	@Override
	public void setQuadSimPosition(double pos) {
		encoderPos.set(pos);
	}

	@Override
	public void setQuadSimVelocity(double vel) {
		encoderVel.set(vel);
	}

	@Override
	public void follow(NAR_EMotor motor) {
		if(!(motor instanceof CANSparkMax)) {
			throw new RuntimeException("bad follow");
		}
		super.follow((CANSparkMax)motor);
	}

	@Override
	public NAR_EMotor getMotor() {
		return this;
	}
}
