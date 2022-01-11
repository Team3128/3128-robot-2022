package frc.team3128.common;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// Daniel - establish convention for these methods and ensure that subclasses implement them
public interface NAR_EMotor extends MotorController {

    /**
     * @return Current encoder position in encoder ticks
     */
    public double getSelectedSensorPosition();

    /**
     * @return Current encoder velocity in encoder ticks per 100ms
     */
    public double getSelectedSensorVelocity();

    /**
     * @return Motor output in volts
     */
    public double getMotorOutputVoltage();

    /**
     * @param encPos Encoder position in encoder ticks
     */
    public void setEncoderPosition(double encPos);

    /**
     * 
     */
    public void setQuadSimPosition(double pos);
    public void setQuadSimVelocity(double vel);

    /**
     * @param motor NAR_EMotor to follow
     */
    public void follow(NAR_EMotor motor);

    // This is stupid and needs a better solution
    public NAR_EMotor getMotor();
}
