package frc.team3128.subsystems;

import frc.team3128.ConstantsInt;
import static frc.team3128.Constants.HoodConstants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.utility.interpolation.InterpolatingDouble;
import net.thefletcher.revrobotics.SparkMaxRelativeEncoder;
import net.thefletcher.revrobotics.enums.IdleMode;
import net.thefletcher.revrobotics.enums.MotorType;
import net.thefletcher.revrobotics.enums.PeriodicFrame;

public class Hood extends PIDSubsystem {

    private static Hood instance;
    private NAR_CANSparkMax m_hoodMotor;
    private SparkMaxRelativeEncoder m_encoder;
    
    private double tolerance = TOLERANCE_MIN;
    
    public Hood() {
        super(new PIDController(kP, kI, kD));

        configMotors();
        configEncoder();
    }

    public static synchronized Hood getInstance() {
        if (instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    private void configMotors() {
        m_hoodMotor = new NAR_CANSparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
        m_hoodMotor.setSmartCurrentLimit(HOOD_CURRENT_LIMIT);
        m_hoodMotor.enableVoltageCompensation(12.0);
        m_hoodMotor.setIdleMode(IdleMode.kBrake);

        m_hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 15);
        m_hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 45);
        m_hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
        m_hoodMotor.setControlFramePeriodMs(20);
    }

    private void configEncoder() {
        m_encoder = (SparkMaxRelativeEncoder) m_hoodMotor.getEncoder();
        m_encoder.setPositionConversionFactor(ENC_POSITION_CONVERSION_FACTOR);
    }

    public void startPID(double angle) {
        // angle = ConstantsInt.ShooterConstants.SET_ANGLE; // uncomment for interpolation
        super.setSetpoint(angle);
        getController().setTolerance(tolerance);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // ff needs a fix b/c "degrees" are fake right now (min angle was given an arbitrary number, not the real number)
        double ff = kF * Math.cos(Units.degreesToRadians(setpoint)); // ff keeps the hood at steady to counteract Fg (gravity)
        double voltageOutput = output + ff;

        m_hoodMotor.set(MathUtil.clamp(voltageOutput / 12.0, -1, 1));
    }

    public void stop() {
        m_hoodMotor.set(0);
    }

    /**
     * Encoder returns 0 deg when at min angle.
     */
    public void zeroEncoder() {
        m_hoodMotor.setEncoderPosition(0);
    }

    @Override
    public double getMeasurement() {
        return m_hoodMotor.getSelectedSensorPosition() + MIN_ANGLE;
    }

    /**
     * Calculates the preferred hood angle for shooting at distance dist
     * Uses InterpolatingTreeMap in Constants
     */
    public double calculateAngleFromDist(double dist) {
        return MathUtil.clamp(
            hoodAngleMap.getInterpolated(new InterpolatingDouble(dist)).value, 
            MIN_ANGLE, MAX_ANGLE);
    }
}

