package frc.team3128.common.hardware.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.team3128.Robot;
import frc.team3128.common.NAR_EMotor;

//import frc.team3128.common.NARMotor;
//import frc.team3128.common.NARTalon;

public class NAR_TalonFX extends WPI_TalonFX implements NAR_EMotor {
    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;
	private NAR_TalonSRX fakeMotor;

	public static int simMotorID = 3128;

	/**	 
	 * @param deviceNumber device id
	 */
	public NAR_TalonFX(int deviceNumber) {
		super(deviceNumber);
		if(Robot.isSimulation()){
			fakeMotor = new NAR_TalonSRX(simMotorID);
			simMotorID--;
		}

		configVoltageCompSaturation(12, 10);
		enableVoltageCompensation(true);
	}

	public void set(ControlMode controlMode, double outputValue) {
		if(fakeMotor != null){
			fakeMotor.set(controlMode, outputValue);
		}else{
			if (outputValue != prevValue || controlMode != prevControlMode) {
				super.set(controlMode, outputValue);
				prevValue = outputValue;
			}
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
		if(fakeMotor != null)
			return fakeMotor.getSelectedSensorPosition();	
		else
			return super.getSelectedSensorPosition();
	}

	@Override
	public double getSelectedSensorVelocity() {
		if(fakeMotor != null)
			return fakeMotor.getSelectedSensorVelocity();	
		else
			return super.getSelectedSensorVelocity();
	}

	@Override
	public double getMotorOutputVoltage(){
		if(fakeMotor != null)
			return fakeMotor.getMotorOutputVoltage();	
		else
			return super.getSelectedSensorPosition(); // what the hell
	}

	@Override
	public void setEncoderPosition(double n) {
		if(fakeMotor != null)
			fakeMotor.setEncoderPosition(n);	
		else
			super.setSelectedSensorPosition(n);
	}

	@Override
	public void setQuadSimPosition(double pos) {
		fakeMotor.setQuadSimPosition((int)pos);
	}

	@Override
	public void setQuadSimVelocity(double vel) {
		fakeMotor.setQuadSimVelocity((int)vel);
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
		if(fakeMotor != null)
			return fakeMotor;
		else
			return this;
	}
}