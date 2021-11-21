package frc.team3128.common;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public abstract class NAR_CTREMotor extends BaseMotorController implements NAR_EMotor{
    public NAR_CTREMotor(int deviceNumber, String model) {
        super(deviceNumber, model);
        //TODO Auto-generated constructor stub
    }

    @Override
    public void follow(NAR_EMotor motor){

    }
}
