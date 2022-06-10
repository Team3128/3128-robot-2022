package frc.team3128.subsystems;

import frc.team3128.Constants;
import frc.team3128.ConstantsInt;
import frc.team3128.Robot;

import static frc.team3128.Constants.HoodConstants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import frc.team3128.common.infrastructure.NAR_PIDSubsystem;
import frc.team3128.common.utility.interpolation.InterpolatingDouble;
import net.thefletcher.revrobotics.SparkMaxRelativeEncoder;
import net.thefletcher.revrobotics.enums.IdleMode;
import net.thefletcher.revrobotics.enums.MotorType;
import net.thefletcher.revrobotics.enums.PeriodicFrame;

public class Hood extends NAR_PIDSubsystem {

    private static Hood instance;
    private NAR_CANSparkMax m_hoodMotor;
    //private NAR_TalonFX m_hoodMotor;
    private SparkMaxRelativeEncoder m_encoder;
    
    private double tolerance = TOLERANCE_MIN;

    private double time;
    private double prevTime;

    private SingleJointedArmSim singleJointedArmSim;

    public static synchronized Hood getInstance() {
        if(instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    public Hood() {
        super(new PIDController(kP, kI, kD), PLATEAU_COUNT);

        configMotors();
        configEncoder();

        if(Robot.isSimulation()) {
            singleJointedArmSim = new SingleJointedArmSim(
                DCMotor.getNeo550(1), 
                HOOD_SHOOTER_GEAR_RATIO, 
                0.054195108, //TODO Find this (COMMUNISM OVER CAPITALISM) 
                0.2400046, 
                Units.degreesToRadians(3), 
                Units.degreesToRadians(32), 
                0.795601019, true);
        }
    }

    private void configMotors() {
        m_hoodMotor = new NAR_CANSparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
        m_hoodMotor.setSmartCurrentLimit(HOOD_CURRENT_LIMIT);
        m_hoodMotor.enableVoltageCompensation(12.0);
        m_hoodMotor.setIdleMode(IdleMode.kBrake);

        m_hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 19);
        m_hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
        m_hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 53);
    }

    private void configEncoder() {
        m_encoder = (SparkMaxRelativeEncoder) m_hoodMotor.getEncoder();
        m_encoder.setPositionConversionFactor(ENC_POSITION_CONVERSION_FACTOR);
    }

    public void setSpeed(double speed) {
        m_hoodMotor.set(speed);
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

    public void startPID(double angle) {
        tolerance = TOLERANCE_MIN;
        // angle = ConstantsInt.ShooterConstants.SET_ANGLE;
        super.setSetpoint(angle);
        super.resetPlateauCount();
        getController().setTolerance(tolerance);
    }

    /**
     * Attempts to PID to minimum angle. Will likely be replaced by full homing routine once limit switch is added.
     */
    public void zero() {
        startPID(MIN_ANGLE);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double ff = kF * Math.cos(Units.degreesToRadians(setpoint));
        double voltageOutput = output + ff;

        time = RobotController.getFPGATime() / 1e6;
        if (tolerance < TOLERANCE_MAX) {
            tolerance += (time - prevTime) * (TOLERANCE_MAX - TOLERANCE_MIN) / TIME_TO_MAX_TOLERANCE;
            getController().setTolerance(tolerance);
        }

        checkPlateau(setpoint, tolerance);

        m_hoodMotor.set(voltageOutput / 12.0);

        prevTime = time;

        // SmartDashboard.putNumber("Hood voltage", voltageOutput);
        // SmartDashboard.putNumber("Hood percentage output", voltageOutput / 12.0);

    }

    @Override
    public double getMeasurement() {
        return m_hoodMotor.getSelectedSensorPosition() + MIN_ANGLE;
    }

    public double calculateAngleFromDistance(double dist) {
        // double yay = 7.62717674e-8*dist*dist*dist*dist - 3.20341423e-5*dist*dist*dist + 5.01101227e-3*dist*dist - 2.624432553e-0*dist + 2.20193191e1;

        return MathUtil.clamp(hoodAngleMap.getInterpolated(new InterpolatingDouble(dist)).value, MIN_ANGLE, MAX_ANGLE);
    }

    @Override
    public void simulationPeriodic() {
        singleJointedArmSim.setInputVoltage(
            m_hoodMotor.getMotorOutputVoltage()
        );

        singleJointedArmSim.update(0.02);

        double angle = singleJointedArmSim.getAngleRads()/(Math.PI) * 360;

        m_encoder.setPosition(angle);
        m_hoodMotor.setSimVelocity(singleJointedArmSim.getVelocityRadPerSec() / Constants.ConversionConstants.SPARK_ENCODER_RESOLUTION);
        m_hoodMotor.setSimPosition(angle);

        //SmartDashboard.putNumber("Hood Position", m_encoder.getPosition());
    }
}

