package frc.team3128.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ControlType;

import edu.wpi.first.hal.SimDevice;
import frc.team3128.Robot;
import frc.team3128.common.NAR_CTREMotor;
import frc.team3128.common.NAR_EMotor;

//import frc.team3128.common.NARMotor;
//import frc.team3128.common.NARTalon;

public class NAR_TalonFX extends WPI_TalonFX implements NAR_EMotor{
    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;
	private WPI_TalonSRX fakeMotor;
	private TalonSRXSimCollection motorSim;

	public static int simMotorID = 3128;

	/**	 * 
	 * @param deviceNumber device id
	 */
	public NAR_TalonFX(int deviceNumber) {
		super(deviceNumber);

		if(Robot.isSimulation()){
			fakeMotor = new WPI_TalonSRX(simMotorID);
			motorSim = new TalonSRXSimCollection(fakeMotor);
			simMotorID--;
		}

		configVoltageCompSaturation(12, 10);
		enableVoltageCompensation(true);
	}

	// @Override
	// public void set(ControlMode controlMode, double outputValue) {
	// 	if (outputValue != prevValue || controlMode != prevControlMode) {
	// 		super.set(controlMode, outputValue);
	// 		prevValue = outputValue;
	// 	}
	// }
	
	public double getSetpoint() {
		return prevValue;
	}

	@Override
	public double getEncoderPosition() {
		return super.getSelectedSensorPosition();
	}

	@Override
	public void setEncoderPosition(double n) {
		super.setSelectedSensorPosition(n);
	}

	@Override
	public void follow(NAR_EMotor motor) {
		if(!(motor instanceof IMotorController)) {
			throw new RuntimeException("bad follow");
		}
		super.follow((IMotorController)motor);
	}

	@Override
	public void setRawSimPosition(double pos) {
		motorSim.setQuadratureRawPosition((int)pos);
	}

	@Override
	public void setRawSimVelocity(double vel) {
		motorSim.setQuadratureVelocity((int)vel);
	}
}