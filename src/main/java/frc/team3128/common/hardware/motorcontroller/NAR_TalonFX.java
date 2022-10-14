package frc.team3128.common.hardware.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotBase;
import frc.team3128.common.hardware.motor.NAR_Motor;

public class NAR_TalonFX extends WPI_TalonFX {

    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;
	private TalonFXSimCollection motorSim;
	private NAR_Motor motor;

	/**	 
	 * @param deviceNumber device id
	 */
	public NAR_TalonFX(int deviceNumber, NAR_Motor motor) {
		super(deviceNumber);
		this.motor = motor;

		if(RobotBase.isSimulation()){
			motorSim = getTalonFXSimCollection();
		}

		configVoltageCompSaturation(12, 10);
		enableVoltageCompensation(true);
	}
  
  public NAR_TalonFX(int deviceNumber) {
		this(deviceNumber, null);
	}

	@Override
	public void set(double speed) {
		set(ControlMode.PercentOutput, speed);
	}
  
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