package frc.team3128.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3128.Constants.ClimberConstants.*;
import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.*;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;

/**
 * Class for the Climber Subsystem 
 */

public class Climber extends SubsystemBase {

    private static Climber instance;

    private DoubleSolenoid m_climberSolenoid;
    private NAR_TalonFX m_leftMotor, m_rightMotor;

    public Climber() {

        configMotors();
        configPneumatics();

        resetLeftEncoder();
    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    /**
     * Initializes motors and sets up CAN frame periods
     */
    private void configMotors() {
        m_leftMotor = new NAR_TalonFX(CLIMBER_MOTOR_LEFT_ID);
        m_rightMotor = new NAR_TalonFX(CLIMBER_MOTOR_RIGHT_ID);

        m_leftMotor.setInverted(true);
        // right follows left - left is the one we use for encoder/distance calculations
        m_rightMotor.follow(m_leftMotor); 
        m_rightMotor.setInverted(InvertType.OpposeMaster);
        
        m_leftMotor.setNeutralMode(CLIMBER_NEUTRAL_MODE);
        m_rightMotor.setNeutralMode(CLIMBER_NEUTRAL_MODE);

        m_leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 45);
        m_leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 45);

        m_rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 45);
        m_rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 45);

    }

    /**
     * Initializes pneumatics
     */
    private void configPneumatics() {
        m_climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                CLIMBER_SOLENOID_FORWARD_CHANNEL_ID, 
                                                CLIMBER_SOLENOID_BACKWARD_CHANNEL_ID);

        retractPiston();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Encoder", getCurrentTicks());
    }

    /**
     * Extends climber arms at regular power
     */
    public void bothExtend() {
        m_rightMotor.set(CLIMBER_POWER);
        m_leftMotor.set(CLIMBER_POWER);
    }

    /**
     * Retracts/lowers climber arms at regular power
     */
    public void bothRetract() {
        m_rightMotor.set(-CLIMBER_POWER);
        m_leftMotor.set(-CLIMBER_POWER);
    }

    /**
     * Extends climber arms at a slower power for resetting purposes 
     */
    public void bothManualExtend() {
        m_rightMotor.set(MANUAL_POWER);
        m_leftMotor.set(MANUAL_POWER);
    }

    /**
     * Retracts/lowers climber arms at a slower power for resetting purposes 
     */
    public void bothManualRetract() {
        m_rightMotor.set(-MANUAL_POWER);
        m_leftMotor.set(-MANUAL_POWER);
    }

    /**
     * Halts climber arms
     */
    public void bothStop() {
        m_rightMotor.set(0);
        m_leftMotor.set(0);
    }

    /**
     * Extends climber pistons to diagonal state
     */
    public void extendPiston(){
        m_climberSolenoid.set(kForward);
    }

    /**
     * Retracts climber pistons to upright state
     */
    public void retractPiston(){
        m_climberSolenoid.set(kReverse);
    }
    
    /**
     * Translates distance for climber arms to move in inches to encoder counts
     * @param distance Distance to extend/retract in inches
     * @return Corresponding encoder counts
     */
    public double getDesiredTicks(double distance) {
        return distance * (FALCON_ENCODER_RESOLUTION * CLIMBER_GEAR_RATIO) / (AXLE_DIAMETER * Math.PI);
    }

    /**
     * @return Current encoder count position of left (leader) motor
     */
    public double getCurrentTicks() {
        return m_leftMotor.getSelectedSensorPosition();
    }

    /**
     * Resets left (leader) climber encoder 
     */
    public void resetLeftEncoder() {
        m_leftMotor.setEncoderPosition(0);
    }
}