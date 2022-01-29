package frc.team3128.subsystems; 

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team3128.hardware.motorcontroller.NAR_TalonSRX;
import frc.team3128.infrastructure.NAR_EMotor;
import frc.team3128.Constants;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Hopper extends SubsystemBase {

    private static Hopper instance;

    private NAR_EMotor m_hopper;
    private DoubleSolenoid m_hpiston;

    private boolean isEjected;

    public Hopper() {
        configMotors();
        configPneumatics();
        configSensors();
      
        isEjected = true;
    }

    public static synchronized Hopper getInstance() {
        if (instance == null) instance = new Hopper();
        return instance;
    }

    private void configMotors() {
        m_hopper = new NAR_TalonSRX(Constants.HopperConstants.HOPPER_MOTOR_ID);
    }

    private void configPneumatics() {
        m_hpiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.HopperConstants.HOPPER_SOLENOID_FORWARD_CHANNEL_ID, Constants.HopperConstants.HOPPER_SOLENOID_BACKWARD_CHANNEL_ID);
    }
    private void configSensors() {
        // m_bottom = new DigitalInput(Constants.HopperConstants.BOTTOM_SENSOR_ID);
        // m_top = new DigitalInput(Constants.HopperConstants.TOP_SENSOR_ID);
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
     * Ejects the piston gate
     */
    public void ejectPistonGate(){
        m_hpiston.set(kForward);
        isEjected = true;
    }

    /**
     * Retracts the piston gate
     */
    public void retractPistonGate(){
        m_hpiston.set(kReverse);
        isEjected = false;
    }

    /**
     * Stops the piston gate - emergency stop/power off
     */
    public void turnPistonOff() {
        m_hpiston.set(kOff); 
    }

    /**
     * Runs the hopper using the HOPPER_MOTOR_POWER constant
     */
    public void runHopper() {
        m_hopper.set(Constants.HopperConstants.HOPPER_MOTOR_POWER);
    }

    /**
     * Stops the hopper - sets power to 0
     */
    public void stopHopper() {
        m_hopper.set(0);
    }
}