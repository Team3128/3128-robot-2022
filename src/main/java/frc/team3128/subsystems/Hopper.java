package frc.team3128.subsystems; 

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants.HopperConstants;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;

public class Hopper extends SubsystemBase {

    private static Hopper instance;

    private NAR_TalonSRX m_hopper;
    //private DoubleSolenoid m_hopperSolenoid;
    private Encoder m_encoder;

    private boolean isEjected;

    public Hopper() {
        configMotors();
        configPneumatics();
        configSensors();
        configEncoders();
        // resetEncoder();
      
        isEjected = true;
    }

    public static synchronized Hopper getInstance() {
        if (instance == null) 
            instance = new Hopper();
        return instance;
    }

    private void configMotors() {
        m_hopper = new NAR_TalonSRX(HopperConstants.HOPPER_MOTOR_ID);
    }

    private void configPneumatics() {
    //     m_hopperSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, HopperConstants.HOPPER_SOLENOID_FORWARD_CHANNEL_ID, HopperConstants.HOPPER_SOLENOID_BACKWARD_CHANNEL_ID);
     }

    private void configEncoders() {
        m_encoder = new Encoder(HopperConstants.HOPPER_DIO_PIN1, HopperConstants.HOPPER_DIO_PIN2);
        m_encoder.setDistancePerPulse(2.5*Math.PI);
        m_encoder.setReverseDirection(true);
    }

    private void configSensors() {
        // m_bottom = new DigitalInput(HopperConstants.BOTTOM_SENSOR_ID);
        // m_top = new DigitalInput(HopperConstants.TOP_SENSOR_ID);
    }

    // public boolean getTop() {
    //     return !m_top.get(); // .get() is inverted
    // }

    // public boolean getBottom() {
    //     return !m_bottom.get(); // .get() is inverted
    // }

    /**
     * Tracks gate ejected state
     * @return true if gate ejected, false if retracted
     */

    public boolean getEjected() {
        return isEjected;
    }

    /**
     * @return true if hopper has reversed to desired distance, false if retracted
     */
    public boolean isReversed() {
        return m_encoder.getDistance() <= HopperConstants.HOPPER_MAX_REVERSE_DISTANCE;
    }

    /**
     * Ejects the piston gate
     */
    public void ejectPistonGate(){
    //     m_hopperSolenoid.set(kForward);
    //     isEjected = true;
    }

    /**
     * Retracts the piston gate
     */
    // public void retractPistonGate(){
    //     m_hopperSolenoid.set(kReverse);
    //     isEjected = false;
    // }

    /**
     * Stops the piston gate - emergency stop/power off
     */
    // public void turnPistonOff() {
    //   m_hopperSolenoid.set(kOff); 
    // }

    /**
     * Runs the hopper using the HOPPER_MOTOR_POWER constant
     */
    public void runHopper() {
        m_hopper.set(HopperConstants.HOPPER_MOTOR_POWER);

    }

    public void reverseHopper() {
        m_encoder.reset();
        m_hopper.set(HopperConstants.REVERSE_HOPPER_MOTOR_POWER); //change later
    }

    /**
     * Stops the hopper - sets power to 0
     */
    public void stopHopper() {
        m_hopper.set(0);
    }

    public void resetMotorEncoder(){
        m_hopper.setSelectedSensorPosition(0);
    }

    public void setNeutralMode(NeutralMode mode) {
        m_hopper.setNeutralMode(mode);
    }
}