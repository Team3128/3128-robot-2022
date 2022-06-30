package frc.team3128.subsystems; 

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import static frc.team3128.Constants.HopperConstants.*;

/**
 * Class for the Hopper Subsystem 
 * (most reliable subsystem eyyyyyyy)
 */

public class Hopper extends SubsystemBase {

    private static Hopper instance;

    private NAR_TalonSRX m_hopper1, m_hopper2;

    private Encoder m_encoder;

    static {
        instance = new Hopper();
    }
    
    private Hopper() {
        configMotors();
        configEncoders();
    }

    public static Hopper getInstance() {
        return instance;
    }

    /**
     * Initializes motors and sets up CAN frame periods
     */
    private void configMotors() {
        m_hopper1 = new NAR_TalonSRX(HOPPER_MOTOR_ID);
        m_hopper2 = new NAR_TalonSRX(HOPPER_MOTOR_2_ID);

        m_hopper1.setNeutralMode(NeutralMode.Coast);
        m_hopper2.setNeutralMode(NeutralMode.Coast);

        m_hopper1.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 45);
        m_hopper1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 45);
        m_hopper1.setControlFramePeriod(ControlFrame.Control_3_General, 20);

        m_hopper2.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 45);
        m_hopper2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 45);
        m_hopper2.setControlFramePeriod(ControlFrame.Control_3_General, 20);
    }

    /**
     * Initializes hopper encoder 
     */
    private void configEncoders() {
        m_encoder = new Encoder(HOPPER_DIO_PIN1, HOPPER_DIO_PIN2);
        m_encoder.setReverseDirection(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hopper Enc", m_encoder.getDistance());
    }

    /**
     * Runs the hopper motors using the default power
     */
    public void runHopper() {
        m_hopper1.set(HOPPER_MOTOR_POWER);
        m_hopper2.set(HOPPER_MOTOR_2_POWER);
    }

    /**
     * Runs the hopper motors using the parameter power
     */
    public void runHopper(double power) {
        m_hopper1.set(power);
        m_hopper2.set(power + 0.1);
    }

    /**
     * Runs hopper motor back at parameter power
     */
    public void reverseHopper(double power) {
        m_hopper1.set(power);
    }

    /**
     * Stops the hopper - sets power to 0
     */
    public void stopHopper() {
        m_hopper1.set(0);
        m_hopper2.set(0);
    }

    /**
     * Resets hopper encoder
     */
    public void resetEncoder(){
        m_encoder.reset();
    }

    /**
     * Gets hopper distance using encoder
     */
    public double getHopperDistance() {
        return m_encoder.getDistance();
    }

}