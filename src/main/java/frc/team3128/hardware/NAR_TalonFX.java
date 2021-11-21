package frc.team3128.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ControlType;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.SpeedController;
import frc.team3128.Robot;
import frc.team3128.common.NAR_EMotor;

//import frc.team3128.common.NARMotor;
//import frc.team3128.common.NARTalon;

public class NAR_TalonFX extends WPI_TalonFX implements NAR_EMotor{
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
	}

	@Override
	public NAR_EMotor getMotor(){
		if(fakeMotor != null)
			return fakeMotor;
		else
			return this;
	}
	
	public double getSetpoint() {
		return prevValue;
	}

	@Override
	public double getMotorOutputVoltage(){
		return getMotor().getMotorOutputVoltage();
	}

	@Override
	public double getSelectedSensorPosition(){
		return getMotor().getSelectedSensorPosition();
	}

	@Override 
	public double getSelectedSensorVelocity(){
		return getMotor().getSelectedSensorVelocity();
	}

	@Override
	public void setEncoderPosition(double n) {
		if(fakeMotor != null)
			fakeMotor.setSelectedSensorPosition(n);
		else
			setSelectedSensorPosition(n);
	}

	@Override
	public void follow(NAR_EMotor motor) {
		if(!(motor instanceof IMotorController)) {
			throw new RuntimeException("bad follow");
		}
		super.follow((IMotorController)motor);
	}

	// Only called in sim environments, so no need to check whether or not fakeMotor is real
	@Override
	public void setRawSimPosition(double pos) {
		fakeMotor.setRawSimPosition(pos);
	}

	@Override
	public void setRawSimVelocity(double vel) {
		fakeMotor.setRawSimVelocity(vel);
	}
}