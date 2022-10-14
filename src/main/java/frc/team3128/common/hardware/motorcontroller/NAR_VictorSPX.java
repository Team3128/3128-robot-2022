
package frc.team3128.common.hardware.motorcontroller;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.team3128.common.hardware.motor.NAR_Motor;

// To be simmed in the next implementation

public class NAR_VictorSPX extends WPI_VictorSPX {
    
    private double prevValue = 0;
    private ControlMode prevControlMode = ControlMode.Disabled;
    private NAR_Motor motor;

    /**
     * @param deviceNumber device id
     */
    public NAR_VictorSPX(int deviceNumber, NAR_Motor motor){
        super(deviceNumber);
        this.motor = motor;
    }

    public NAR_VictorSPX(int deviceNumber) {
		this(deviceNumber, null);
	}

    @Override
    public void set(double speed) {
        set(ControlMode.PercentOutput, speed);
    }

    @Override
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
}
    

