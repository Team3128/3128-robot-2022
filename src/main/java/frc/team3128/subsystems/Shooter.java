package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import static frc.team3128.Constants.ShooterConstants.*;
import frc.team3128.ConstantsInt;
import static frc.team3128.Constants.ConversionConstants.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import frc.team3128.common.infrastructure.NAR_PIDSubsystem;
import frc.team3128.common.utility.Log;
import frc.team3128.common.utility.interpolation.InterpolatingDouble;

public class Shooter extends NAR_PIDSubsystem {
    
    public enum ShooterState {

        OFF(0, 0, 0, 0),
        UPPERHUB(0, HIGH_kP, HIGH_kI, HIGH_kD),
        LOWERHUB(1250, LOW_kP, LOW_kI, LOW_kD);

        public double shooterRPM;
        public double kP;
        public double kI;
        public double kD;

        private ShooterState(double RPM, double kP, double kI, double kD) {
            this.shooterRPM = RPM;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

    }

    private static Shooter instance;

    //Motors
    private NAR_TalonFX m_leftShooter;
    private NAR_TalonFX m_rightShooter;

    private ShooterState shooterState = ShooterState.OFF;
    private double time = 0, preTime = 0;
    private double thresholdPercent = RPM_THRESHOLD_PERCENT;

    private FlywheelSim m_shooterSim;

    // private SimpleMotorFeedforward lowFF = new SimpleMotorFeedforward(ShooterConstants.LOW_kS, ShooterConstants.LOW_kV, ShooterConstants.LOW_kA);
    // private SimpleMotorFeedforward highFF = new SimpleMotorFeedforward(ShooterConstants.HIGH_kS, ShooterConstants.HIGH_kV, ShooterConstants.HIGH_kA);

    public Shooter() {
        super(new PIDController(HIGH_kP, HIGH_kI, HIGH_kD), PLATEAU_COUNT);
    
        configMotors();

        //Robot is a simulation
        if(RobotBase.isSimulation()){
            m_shooterSim = new FlywheelSim(
                SHOOTER_CHAR, SHOOTER_GEARBOX, SHOOTER_GEARING 
            );
        }
    }

    public static synchronized Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private void configMotors() {
        m_leftShooter = new NAR_TalonFX(LEFT_SHOOTER_ID);
        m_rightShooter = new NAR_TalonFX(RIGHT_SHOOTER_ID);

        m_leftShooter.setInverted(false);
        m_rightShooter.setInverted(true);

        m_rightShooter.follow(m_leftShooter);

        // set CAN status frame periods
        m_rightShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        m_rightShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        m_rightShooter.setControlFramePeriod(ControlFrame.Control_3_General, 45);
        
        m_leftShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 15);
        m_leftShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 15);
        m_leftShooter.setControlFramePeriod(ControlFrame.Control_3_General, 20);

        m_leftShooter.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 30, 0.1));
    }

    /**
     * Sets the Shooter State for the shooter
     * @param state Desired Shooter State
     */
    public void setState(ShooterState state) {
        this.shooterState = state;
        
        m_controller.setPID(shooterState.kP, shooterState.kI, shooterState.kD);
        Log.info("Shooter", "Set state to " + state.toString());
        Log.info("Shooter", "Set kP to " + String.valueOf(state.kP));
        Log.info("Shooter", "Set kI to " + String.valueOf(state.kI));
        Log.info("Shooter", "Set kD to " + String.valueOf(state.kD));
    }

    /**
     * Begins the PID loop to achieve the desired RPM with the currently set Shooter State
     */
    public void startPID() {
        thresholdPercent = RPM_THRESHOLD_PERCENT;
        super.setSetpoint(shooterState.shooterRPM);  
        super.resetPlateauCount();
        getController().setTolerance(RPM_THRESHOLD_PERCENT * shooterState.shooterRPM);
    }

    /**
     * Begins the PID loop to achieve the desired RPM with the currently set Shooter State
     */
    public void startPID(double rpm) {
        thresholdPercent = RPM_THRESHOLD_PERCENT;
        // rpm = ConstantsInt.ShooterConstants.SET_RPM;
        super.setSetpoint(rpm);
        getController().setTolerance(RPM_THRESHOLD_PERCENT * rpm);
    }

    /**
     * Sets the state using {@link #setState(ShooterState)} and begins the PID loop to achieve it using {@link #startPID()}
     * @param state Desired Shooter State
     */
    public void beginShoot(ShooterState state) {
        setState(state);
        startPID();
    }

    /**
     * Sets the state using {@link #setState(ShooterState)} and begins the PID loop to achieve it using {@link #startPID()}
     * @param state Desired Shooter State
     */
    public void beginShoot(double rpm) {
        startPID(rpm);
    }

    public void stopShoot() {
        super.resetPlateauCount();
        setSetpoint(0);
    }

    /**
     * @return The Shooter State
     */
    public ShooterState getState() {
        return shooterState;
    }

    @Override
    public double getMeasurement() {
        return m_leftShooter.getSelectedSensorVelocity() * FALCON_NUpS_TO_RPM;
    }

    /**
     * Use the raw voltage output from the PID loop, add a feed forward component, and convert it to a percentage of total
     * possible voltage to apply to the motors.
     * 
     * @param output Output from the PID Loop (RPM)
     * @param setpoint The desired setpoint RPM for the PID Loop (RPM)
     */
    @Override
    protected void useOutput(double output, double setpoint) {
        double ff = kF * setpoint;
        // if (shooterState == ShooterState.LOWERHUB)
        //     ff = lowFF.calculate(setpoint);
        // else if (shooterState == ShooterState.UPPERHUB)
        //     ff = highFF.calculate(setpoint);
        double voltageOutput = output + ff;

        time = RobotController.getFPGATime() / 1e6;
        if (thresholdPercent < RPM_THRESHOLD_PERCENT_MAX) {
            thresholdPercent += (time - preTime) * (RPM_THRESHOLD_PERCENT_MAX - RPM_THRESHOLD_PERCENT) / TIME_TO_MAX_THRESHOLD;
            getController().setTolerance(thresholdPercent * setpoint);
        }

        checkPlateau(setpoint, thresholdPercent);

        if (setpoint == 0)
            voltageOutput = 0;
            
        m_leftShooter.set(ControlMode.PercentOutput, voltageOutput / 12.0);

        preTime = time;
    }

    @Override
    public void simulationPeriodic() {
        m_shooterSim.setInput(
            m_leftShooter.getMotorOutputVoltage()
        );  
        m_shooterSim.update(0.02);    
        
        m_leftShooter.setSimVelocity(m_shooterSim.getAngularVelocityRadPerSec() * SHOOTER_RADIUS_METERS);
        //m_rightShooter.setQuadSimVelocity(m_shooterSim.getAngularVelocityRadPerSec() * SHOOTER_RADIUS_METERS);
    
        // SmartDashboard.putNumber("test", m_leftShooter.getMotorOutputVoltage()); 
        // SmartDashboard.putString("pogger", String.valueOf(m_shooterSim.getAngularVelocityRadPerSec()));
        SmartDashboard.putNumber("shooter RPM", m_shooterSim.getAngularVelocityRadPerSec() * 60 / (2*Math.PI));
        
    }

    public double calculateMotorVelocityFromDist(double dist) {
        return shooterSpeedsMap.getInterpolated(new InterpolatingDouble(dist)).value;
    }
}

