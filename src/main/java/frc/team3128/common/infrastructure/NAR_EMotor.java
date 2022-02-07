package frc.team3128.common.infrastructure;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface NAR_EMotor extends MotorController {

    /**
     * @return Current encoder position in encoder ticks
     */
    public double getSelectedSensorPosition();

    /**
     * @return Current encoder velocity in encoder ticks per second
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
     * @param pos Position in encoder ticks
     */
    public void setSimPosition(double pos);

    /**
     * @param vel Velocity in encoder ticks / second
     */
    public void setSimVelocity(double vel);

    /**
     * @param motor NAR_EMotor to follow 
     * 
     * Restrict this to only follow motor controllers of the same type in subclasses
     */
    public void follow(NAR_EMotor motor);
}
