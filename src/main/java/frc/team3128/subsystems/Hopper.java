package frc.team3128.subsystems; 

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Robot;
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

    private Timer timer;

    private double prevAngularVelocity;
    
    private FlywheelSim m_flywheelSim;

    private FlywheelSim noBallSim;

    private FlywheelSim ballSim;

    protected int ballCount;

    public Hopper() {
        configMotors();
        configEncoders();

        timer = new Timer();

        timer.start();

        if(Robot.isSimulation()) {
            noBallSim  = new FlywheelSim(GEARBOX, HOPPER_MOTOR_GEAR_RATIO, HOPPER_MOMENT_OF_INERTIA);
            ballSim = new FlywheelSim(GEARBOX, HOPPER_MOTOR_GEAR_RATIO, HOPPER_MOMENT_OF_INERTIA_BALL);
            m_flywheelSim = noBallSim;
        }
    }

    public static synchronized Hopper getInstance() {
        if (instance == null) 
            instance = new Hopper();
        return instance;
    }

    /**
     * Initializes motors and sets up CAN frame periods
     */
    private void configMotors() {
        m_hopper1 = new NAR_TalonSRX(HOPPER_MOTOR_ID);
        // m_hopper2 = new NAR_TalonSRX(HOPPER_MOTOR_2_ID);

        m_hopper1.setNeutralMode(NeutralMode.Coast);
        // m_hopper2.setNeutralMode(NeutralMode.Coast);

        m_hopper1.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 45);
        m_hopper1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 45);
        m_hopper1.setControlFramePeriod(ControlFrame.Control_3_General, 20);

        // m_hopper2.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 45);
        // m_hopper2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 45);
        // m_hopper2.setControlFramePeriod(ControlFrame.Control_3_General, 20);
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

        double dt = timer.get();

        timer.reset();

        double angularVelocity = m_encoder.getRate() / 4 * 2 * Math.PI;

        double angularAcceleration = (angularVelocity - prevAngularVelocity) / dt;

        double torque = 0.71-(0.71/18734.05) * angularVelocity * 30 / Math.PI;

        double momentOfInertia = torque / angularAcceleration;

        SmartDashboard.putNumber("Hopper Moment of Inertia", momentOfInertia * 10000);
    }

    /**
     * Runs the hopper motors using the default power
     */
    public void runHopper() {
        m_hopper1.set(HOPPER_MOTOR_POWER);
        // m_hopper2.set(HOPPER_MOTOR_2_POWER);
    }

    /**
     * Runs the hopper motors using the parameter power
     */
    public void runHopper(double power) {
        m_hopper1.set(power);
        // m_hopper2.set(power + 0.1);
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
        // m_hopper2.set(0);
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

    public void addBall() {
        ballCount = MathUtil.clamp((ballCount + 1), 0, 2);
    }

    public void resetBallCount() {
        ballCount = 0;
    }

    @Override
    public void simulationPeriodic() {

        FlywheelSim prevFlywheelSim = m_flywheelSim;

        if (ballCount > 0) {
            m_flywheelSim = ballSim;
        }
        else {
            m_flywheelSim = noBallSim;
        }

        if (m_flywheelSim != prevFlywheelSim) {
            m_flywheelSim.setState(VecBuilder.fill(prevFlywheelSim.getAngularVelocityRadPerSec()));
        }
        
        m_flywheelSim.setInputVoltage(
            m_hopper1.getMotorOutputVoltage()
        );
        
        m_flywheelSim.update(0.02);

        SmartDashboard.putNumber("Hopper 1 Motor RPM", m_flywheelSim.getAngularVelocityRPM()/4);
        SmartDashboard.putNumber("Hopper 1 Motor Voltage", m_hopper1.getMotorOutputVoltage());
        // SmartDashboard.putNumber("Hopper 2 Motor Voltage", m_hopper2.getMotorOutputVoltage());
        SmartDashboard.putNumber("Hopper Ball Count", ballCount);
    }

}