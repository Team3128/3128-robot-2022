package frc.team3128.common;

import edu.wpi.first.wpilibj.SpeedController;

public interface NAR_EMotor extends SpeedController{
    public double getSelectedSensorPosition();
    public double getSelectedSensorVelocity();
    public double getMotorOutputVoltage();
    public void setEncoderPosition(double n);
    public void setQuadSimPosition(double pos);
    public void setQuadSimVelocity(double vel);
    public void follow(NAR_EMotor motor);

    // This is stupid and needs a better solution
    public NAR_EMotor getMotor();
}
