package frc.team3128.subsystems;

import frc.team3128.Constants.HoodConstants;

import net.thefletcher.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import net.thefletcher.revrobotics.enums.IdleMode;
import net.thefletcher.revrobotics.enums.MotorType;

public class Hood extends SubsystemBase {

    private static Hood instance;
    private NAR_CANSparkMax m_hoodMotor;
    private SparkMaxRelativeEncoder m_encoder;

    public static synchronized Hood getInstance() {
        if(instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    public Hood() {
        configMotors();
        configEncoder();
    }

    private void configMotors() {
        m_hoodMotor = new NAR_CANSparkMax(HoodConstants.HOOD_MOTOR_ID, MotorType.kBrushless);
        m_hoodMotor.setSmartCurrentLimit(HoodConstants.HOOD_CURRENT_LIMIT);
        m_hoodMotor.enableVoltageCompensation(12.0);
        m_hoodMotor.setIdleMode(IdleMode.kBrake);
    }

    public void configEncoder() {
        m_encoder = (SparkMaxRelativeEncoder) m_hoodMotor.getEncoder();
        m_encoder.setPositionConversionFactor(HoodConstants.ENC_POSITION_CONVERSION_FACTOR);
    }

    public double getAngle() {
        return m_hoodMotor.getSelectedSensorPosition();
    }

    public void setSpeed(double speed) {
        m_hoodMotor.set(speed);
    }

    public void stop() {
        m_hoodMotor.set(0);
    }

    /**
     * Call when the hood is at the minimum angle.
     */
    public void rezero() {
        m_encoder.setPosition(HoodConstants.MIN_ANGLE);
    }

    public void setHome() {
        m_encoder.setPosition(HoodConstants.HOME_ANGLE);
    }

}
