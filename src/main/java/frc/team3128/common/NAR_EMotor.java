package frc.team3128.common;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.SpeedController;

public interface NAR_EMotor extends SpeedController{
    public double getSelectedSensorPosition();
    public double getSelectedSensorVelocity();
    public double getMotorOutputVoltage();
    public SpeedController getMotor();
    public void setEncoderPosition(double n);
    public void setRawSimPosition(double pos);
    public void setRawSimVelocity(double vel);
    public void follow(NAR_EMotor motor);
}
