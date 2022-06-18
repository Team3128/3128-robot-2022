package frc.team3128.common.hardware.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotBase;

public class NAR_TalonSRX extends WPI_TalonSRX {

    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;
	private TalonSRXSimCollection motorSim;
	private NAR_Motor motor;

	/**
	 * @param deviceNumber device id
	 */
	public NAR_TalonSRX(int deviceNumber, NAR_Motor motor) {
		super(deviceNumber);
		this.motor = motor;

		if(RobotBase.isSimulation())
			motorSim = getTalonSRXSimCollection();
			
		enableVoltageCompensation(true);
		configVoltageCompSaturation(12, 10);
	}

	public NAR_TalonSRX(int deviceNumber) {
		this(deviceNumber, null);
	}

	@Override
	public void set(double speed){
		set(ControlMode.PercentOutput, speed);
	}

	@Override
	public void set(ControlMode controlMode, double outputValue) {
		if (outputValue != prevValue || controlMode != prevControlMode) {
			super.set(controlMode, outputValue);
			prevValue = outputValue;
			prevControlMode = controlMode;
		}
	}

	public double getSetpoint() {
		return prevValue;
	}

	public ControlMode getControlMode() {
		return prevControlMode;
	}

	public void setEncoderPosition(double n) {
		setSelectedSensorPosition(n);
	}

	// getInverted() stuff should only be temporary
	public void setSimPosition(double pos) {
		if(super.getInverted()){
			pos *= -1;
		}
		motorSim.setQuadratureRawPosition((int)pos);
	}

	// getInverted() stuff should only be temporary
	public void setSimVelocity(double vel) {
		if(super.getInverted()){
			vel *= -1;
		}
		motorSim.setQuadratureVelocity((int)(vel / 10)); // convert nu/s to nu/100ms
	}

	public double getSelectedSensorVelocity() {
		return super.getSelectedSensorVelocity() * 10; // convert nu/100ms to nu/s
	}
}