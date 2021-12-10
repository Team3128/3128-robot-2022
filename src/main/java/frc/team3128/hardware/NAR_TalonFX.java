package frc.team3128.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class NAR_TalonFX extends NAR_VexController<WPI_TalonFX>{

    public NAR_TalonFX(int deviceNumber) {
		super(deviceNumber);
	}

	@Override
	public void constructReal() {
		this.motorController = new WPI_TalonFX(deviceNumber);
		motorController.enableVoltageCompensation(true);
		motorController.configVoltageCompSaturation(12, 10);
	}
}