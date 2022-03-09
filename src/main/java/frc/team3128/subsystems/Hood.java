package frc.team3128.subsystems;

import frc.team3128.Constants.HoodConstants;

import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import net.thefletcher.revrobotics.enums.MotorType;

public class Hood extends SubsystemBase {

    private static Hood instance;
    private NAR_CANSparkMax m_hoodMotor;
    private SparkMaxRelativeEncoder m_encoder;

    private double angleOffset = 0;

    public static synchronized Hood getInstance() {
        if(instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    public Hood() {
        configMotors();
    }

    private void configMotors() {
        m_hoodMotor = new NAR_CANSparkMax(HoodConstants.HOOD_MOTOR_ID, MotorType.kBrushless);
        m_hoodMotor.setSmartCurrentLimit(HoodConstants.HOOD_CURRENT_LIMIT);

        m_encoder.setPositionConversionFactor(HoodConstants.ENC_POSITION_CONVERSION_FACTOR);
    }

    public double getAngle() {
        return m_hoodMotor.getSelectedSensorPosition() - angleOffset;
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
        angleOffset = getAngle() - HoodConstants.MIN_ANGLE;
    }

    public void setHome() {
        angleOffset = getAngle() - HoodConstants.HOME_ANGLE;
    }

}
