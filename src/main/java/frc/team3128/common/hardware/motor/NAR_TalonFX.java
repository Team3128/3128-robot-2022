package frc.team3128.common.hardware.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.team3128.Robot;
import frc.team3128.common.NAR_EMotor;

//import frc.team3128.common.NARMotor;
//import frc.team3128.common.NARTalon;

public class NAR_TalonFX extends WPI_TalonFX implements NAR_EMotor {
    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;
	private TalonFXSimCollection motorSim;

	public static int simMotorID = 3128;

	/**	 
	 * @param deviceNumber device id
	 */
	public NAR_TalonFX(int deviceNumber) {
		super(deviceNumber);
		if(Robot.isSimulation()){
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
		// if (outputValue != prevValue || controlMode != prevControlMode) {
		// 	super.set(controlMode, outputValue);
		// 	prevValue = outputValue;
		// }
	}
	
	public double getSetpoint() {
		return prevValue;
	}

	@Override
	public double getSelectedSensorPosition() {
		return super.getSelectedSensorPosition();
	}

	@Override
	public double getSelectedSensorVelocity() {
		return super.getSelectedSensorVelocity();
	}

	@Override
	public double getMotorOutputVoltage(){
		return super.getMotorOutputVoltage();
	}

	@Override
	public void setEncoderPosition(double n) {
			super.setSelectedSensorPosition(n);
	}

	@Override
	public void setSimPosition(double pos) {
		motorSim.setIntegratedSensorRawPosition((int)pos);
	}

	@Override
	public void setSimVelocity(double vel) {
		motorSim.setIntegratedSensorVelocity((int)vel);
	}

	@Override
	public void follow(NAR_EMotor motor) {
		if(!(motor instanceof IMotorController)) {
			throw new RuntimeException("bad follow");
		}
		super.follow((IMotorController)motor);
	}
}