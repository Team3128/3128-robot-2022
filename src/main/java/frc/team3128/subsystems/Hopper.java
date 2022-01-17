package frc.team3128.subsystems; 

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motor.NAR_TalonSRX;
import frc.team3128.common.NAR_EMotor;
import frc.team3128.Constants;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DigitalInput;

public class Hopper extends SubsystemBase {

    private static Hopper instance;

    private NAR_EMotor m_hopper;
    private DoubleSolenoid m_hpiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);

    // private DigitalInput m_bottom, m_top;
    private boolean isEjected;

    public Hopper() {
        configMotors();

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

    public boolean getEjected() {
        return isEjected;
    }

    public void ejectPistonGate(){
        m_hpiston.set(kForward);
        isEjected = true;
    }

    public void retractPistonGate(){
        m_hpiston.set(kReverse);
        isEjected = false;
    }

    public void turnPistonOff() {
        m_hpiston.set(kOff); 
    }

    public void runHopper() {
        m_hopper.set(Constants.HopperConstants.HOPPER_MOTOR_POWER);
    }

    public void stopHopper() {
        m_hopper.set(0);
    }

    @Override
    public void simulationPeriodic() {

    }
}