package frc.team3128.common.hardware.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotBase;
import frc.team3128.common.infrastructure.NAR_EMotor;

public class NAR_TalonFX extends WPI_TalonFX implements NAR_EMotor {

    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;
	private TalonFXSimCollection motorSim;

	/**	 
	 * @param deviceNumber device id
	 */
	public NAR_TalonFX(int deviceNumber) {
		super(deviceNumber);

		if(RobotBase.isSimulation()){
			motorSim = getTalonFXSimCollection();
		}

		configVoltageCompSaturation(12, 10);
		enableVoltageCompensation(true);
	}

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
		super.setSelectedSensorPosition(n);
	}

	// getInverted() stuff should only be temporary
	@Override
	public void setSimPosition(double pos) {
		if(super.getInverted()) {
			pos *= -1;
		}
		motorSim.setIntegratedSensorRawPosition((int)pos);
	}

	// getInverted() stuff should only be temporary
	@Override
	public void setSimVelocity(double vel) {
		if(super.getInverted()) {
			vel *= -1;
		}
		motorSim.setIntegratedSensorVelocity((int)(vel/10)); // convert nu/s to nu/100ms
	}

	@Override
	public double getSelectedSensorVelocity() {
		return super.getSelectedSensorVelocity() * 10; // convert nu/100ms to nu/s
	}

	@Override
	public void follow(NAR_EMotor motor) {
		if(!(motor instanceof IMotorController)) {
			throw new RuntimeException("Bad follow: NAR_TalonFX " + getDeviceID() + " attempted to follow non-CTRE motor controller.");
		}
		super.follow((IMotorController)motor);
	}
}