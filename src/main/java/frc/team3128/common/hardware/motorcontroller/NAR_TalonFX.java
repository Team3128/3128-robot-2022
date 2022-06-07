package frc.team3128.common.hardware.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotBase;

public class NAR_TalonFX extends WPI_TalonFX {

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

	public void setEncoderPosition(double n) {
		super.setSelectedSensorPosition(n);
	}

	// getInverted() stuff should only be temporary
	public void setSimPosition(double pos) {
		if(super.getInverted()) {
			pos *= -1;
		}
		motorSim.setIntegratedSensorRawPosition((int)pos);
	}

	// getInverted() stuff should only be temporary
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
}