package frc.team3128.subsystems;

import frc.team3128.Constants.HoodConstants;
import frc.team3128.Constants.ShooterConstants;

import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.infrastructure.NAR_PIDSubsystem;
import net.thefletcher.revrobotics.enums.IdleMode;
import net.thefletcher.revrobotics.enums.MotorType;

public class HoodPID extends NAR_PIDSubsystem {

    private static Hood instance;
    private NAR_CANSparkMax m_hoodMotor;
    private SparkMaxRelativeEncoder m_encoder;
    private double thresholdPercent = HoodConstants.THRESHOLD_PERCENT;
    private double time;
    private double preTime;

    private double angleOffset = 0;

    public static synchronized Hood getInstance() {
        if(instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    public HoodPID() {
        super(new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD), HoodConstants.PLATEAU_COUNT);

        configMotors();
    }

    private void configMotors() {
        m_hoodMotor = new NAR_CANSparkMax(HoodConstants.HOOD_MOTOR_ID, MotorType.kBrushless);
        m_hoodMotor.setSmartCurrentLimit(HoodConstants.HOOD_CURRENT_LIMIT);
        m_hoodMotor.setIdleMode(IdleMode.kBrake);
        zeroEncoder();

        m_encoder.setPositionConversionFactor(HoodConstants.ENC_POSITION_CONVERSION_FACTOR);
    }

    public void setSpeed(double speed) {
        m_hoodMotor.set(speed);
    }

    public void stop() {
        m_hoodMotor.set(0);
    }

    public void zeroEncoder() {
        m_hoodMotor.setEncoderPosition(0);
    }

    public void startPID(double angle) {
        thresholdPercent = ShooterConstants.RPM_THRESHOLD_PERCENT;
        super.setSetpoint(angle);  
        super.resetPlateauCount();
        getController().setTolerance(ShooterConstants.RPM_THRESHOLD_PERCENT * angle);
    }

    public void zero() {
        startPID(0);
        m_hoodMotor.setEncoderPosition(0);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double ff = HoodConstants.kF * setpoint;
        double voltageOutput = output + ff;
        double voltage = RobotController.getBatteryVoltage();
        double percentOutput = voltageOutput / voltage;

        time = RobotController.getFPGATime() / 1e6;
        if (thresholdPercent < HoodConstants.THRESHOLD_PERCENT_MAX) {
            thresholdPercent += (time - preTime) * (ShooterConstants.RPM_THRESHOLD_PERCENT_MAX - ShooterConstants.RPM_THRESHOLD_PERCENT) / ShooterConstants.TIME_TO_MAX_THRESHOLD;
            getController().setTolerance(thresholdPercent * setpoint);
        }

        checkPlateau(setpoint, thresholdPercent);

        percentOutput = MathUtil.clamp(percentOutput, -1, 1);
        percentOutput = (setpoint == 0) ? 0 : percentOutput;

        m_hoodMotor.set(percentOutput);

        preTime = time;
    }

    @Override
    protected double getMeasurement() {
        return m_hoodMotor.getSelectedSensorPosition() + HoodConstants.MIN_ANGLE;
    }

}
