package frc.team3128.common.hardware.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotBase;

public class NAR_TalonSRX extends WPI_TalonSRX {

    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;
	private TalonSRXSimCollection motorSim;

	/**
	 * @param deviceNumber device id
	 */
	public NAR_TalonSRX(int deviceNumber) {
		super(deviceNumber);

		if(RobotBase.isSimulation())
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