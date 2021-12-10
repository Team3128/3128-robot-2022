package frc.team3128.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class NAR_VictorSPX extends NAR_VexController<WPI_VictorSPX> {

    public NAR_VictorSPX(int deviceNumber) {
		super(deviceNumber);
	}

	@Override
	public void constructReal() {
		this.motorController = new WPI_VictorSPX(deviceNumber);
		motorController.enableVoltageCompensation(true);
		motorController.configVoltageCompSaturation(12, 10);
	}
}