package frc.team3128.common.hardware.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.team3128.Robot;
import frc.team3128.common.NAR_EMotor;

public class NAR_TalonSRX extends WPI_TalonSRX implements NAR_EMotor{
    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;
	private TalonSRXSimCollection motorSim;

	/**
	 * 
	 * @param deviceNumber device id
	 */
	public NAR_TalonSRX(int deviceNumber) {
		super(deviceNumber);

		if(Robot.isSimulation())
			motorSim = getTalonSRXSimCollection();
			
		enableVoltageCompensation(true);
		configVoltageCompSaturation(12, 10);
	}

	@Override
	public void set(ControlMode controlMode, double outputValue) {
		if (outputValue != prevValue || controlMode != prevControlMode) {
			super.set(controlMode, outputValue);
			prevValue = outputValue;
		}
	}

	public double getSetpoint() {
		return prevValue;
	}


	@Override
	public void setEncoderPosition(double n) {
		setSelectedSensorPosition(n);
	}

	@Override
	public void setQuadSimPosition(double pos) {
		motorSim.setQuadratureRawPosition((int)pos);
	}

	@Override
	public void setQuadSimVelocity(double vel) {
		motorSim.setQuadratureVelocity((int)vel);
	}

	@Override
	public void follow(NAR_EMotor motor) {
		if(!(motor instanceof IMotorController)) {
			throw new RuntimeException("bad follow");
		}
		super.follow((IMotorController)motor);
	}
	@Override
	public NAR_EMotor getMotor(){
		return this;
	}
}