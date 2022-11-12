package frc.team3128.subsystems;

import static frc.team3128.Constants.HoodConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.common.utility.interpolation.InterpolatingDouble;
import net.thefletcher.revrobotics.SparkMaxRelativeEncoder;
import net.thefletcher.revrobotics.enums.IdleMode;
import net.thefletcher.revrobotics.enums.MotorType;
import net.thefletcher.revrobotics.enums.PeriodicFrame;

/**
 * Class for the Adjustable Hood Subsystem 
 */

public class Hood extends PIDSubsystem {

    private static Hood instance;
    private NAR_CANSparkMax m_hoodMotor;
    private SparkMaxRelativeEncoder m_encoder;
    public DoubleSupplier m_ff;
    public DoubleSupplier m_setpoint;
    
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

    /**
     * Initializes motors and sets up CAN frame periods
     */
    private void configMotors() {
        m_hoodMotor = new NAR_CANSparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
        m_hoodMotor.setSmartCurrentLimit(HOOD_CURRENT_LIMIT); // Neo 550s require current limiting 20 A or below 
        m_hoodMotor.enableVoltageCompensation(12.0);
        
        m_hoodMotor.setIdleMode(IdleMode.kBrake);

        m_hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 15);
        m_hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 45);
        m_hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
        m_hoodMotor.setControlFramePeriodMs(20);
    }

    /**
     * Initializes SparkMax encoder for the hood angle measurements
     */
    private void configEncoder() {
        m_encoder = (SparkMaxRelativeEncoder) m_hoodMotor.getEncoder();
        m_encoder.setPositionConversionFactor(ENC_POSITION_CONVERSION_FACTOR);
        // zeroEncoder();
    }

    public void initShuffleboard() {
        // General Tab
        NAR_Shuffleboard.addData("General", "Hood Setpoint", this::getSetpoint).withPosition(5, 3);
        NAR_Shuffleboard.addData("General", "Hood Angle", this::getMeasurement).withPosition(4, 3);

        // Hood Tab
        NAR_Shuffleboard.addData("Shooter + Hood", "Hood Setpoint", this::getSetpoint).withPosition(6, 0);
        NAR_Shuffleboard.addData("Shooter + Hood", "Hood Angle", this::getMeasurement).withPosition(7, 0);
        NAR_Shuffleboard.addComplex("Shooter + Hood", "Hood", this).withPosition(8,0);
        NAR_Shuffleboard.addComplex("Shooter + Hood", "Hood_PID", m_controller).withPosition(6, 1).withSize(2,2);
        m_ff = NAR_Shuffleboard.debug("Shooter + Hood", "Hood FF", kF,5,1);
        m_setpoint = NAR_Shuffleboard.debug("Shooter + Hood","SET_ANGLE",0,5,2);
    }

    @Override
    public void periodic() {
        NAR_Shuffleboard.put("Shooter + Hood", "Pred Angle", calculateAngleFromDist(LimelightSubsystem.getInstance().calculateShooterDistance()),5,0);
        super.periodic();
        // SmartDashboard.putNumber("Hood Setpoint", getSetpoint());
        // SmartDashboard.putNumber("Hood Angle", getMeasurement());
    }

    /**
     * Begins the PID loop to achieve the desired angle to shoot
     */
    public void startPID(double angle) {
        // angle = m_setpoint.getAsDouble(); // uncomment for interpolation
        super.setSetpoint(angle);
        getController().setTolerance(TOLERANCE_MIN);
    }

    /**
     * Adds feedforward component that counteracts the gravitational force (Fg) 
     * and the raw voltage output from the PID loop
     * and convert it to a percentage of total possible voltage to apply to the motors.
     * 
     * @param output Output from the PID Loop 
     * @param setpoint The desired setpoint angle for the PID Loop (angle)
     */
    @Override
    protected void useOutput(double output, double setpoint) {
        // ff needs a fix b/c "degrees" are fake right now (min angle was given an arbitrary number, not the real number)
        double ff = m_ff.getAsDouble() * Math.cos(Units.degreesToRadians(setpoint)); // ff keeps the hood at steady to counteract Fg (gravity)
        double voltageOutput = output + ff;

        m_hoodMotor.set(MathUtil.clamp(voltageOutput / 12.0, -1, 1));
    }

    /**
     * Stops hood motor
     */
    public void stop() {
        m_hoodMotor.set(0);
    }

    /**
     * Sets the encoder to 0 (corresponding to min angle)
     */
    public void zeroEncoder() {
        m_hoodMotor.setEncoderPosition(0);
    }

    /**
     * Gets current hood angle
     */
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

