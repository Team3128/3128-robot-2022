package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.team3128.Constants.ShooterConstants;
import frc.team3128.Constants.ConversionConstants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import frc.team3128.common.infrastructure.NAR_EMotor;
import frc.team3128.common.infrastructure.NAR_PIDSubsystem;
import frc.team3128.common.utility.Log;

public class Shooter extends NAR_PIDSubsystem {
    
    public enum ShooterState {

        OFF(0),
        LAUNCHPAD(3300),
        UPPERHUB(0),
        LOWERHUB(1250);

        public double shooterRPM;

        private ShooterState(double RPM) {
            this.shooterRPM = RPM;
        }

    }

    private static Shooter instance;

    //Motors
    private NAR_TalonFX m_leftShooter;
    private NAR_TalonFX m_rightShooter;

    private ShooterState shooterState = ShooterState.OFF;
    private double time = 0, preTime = 0;
    private double thresholdPercent = ShooterConstants.RPM_THRESHOLD_PERCENT;

    private FlywheelSim m_shooterSim;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.SHOOTER_KS, ShooterConstants.SHOOTER_KV, ShooterConstants.SHOOTER_KA);


    public Shooter() {
        super(new PIDController(ShooterConstants.SHOOTER_PID_kP, ShooterConstants.SHOOTER_PID_kI, ShooterConstants.SHOOTER_PID_kD), ShooterConstants.PLATEAU_COUNT);
    
        configMotors();

        //Robot is a simulation
        if(RobotBase.isSimulation()){
            m_shooterSim = new FlywheelSim(
                ShooterConstants.SHOOTER_CHAR,
                ShooterConstants.SHOOTER_GEARBOX,
                ShooterConstants.SHOOTER_GEARING 
            );
        }
    }

    public static synchronized Shooter getInstance() {
        if(instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private void configMotors() {
        m_leftShooter = new NAR_TalonFX(ShooterConstants.LEFT_SHOOTER_ID);
        m_rightShooter = new NAR_TalonFX(ShooterConstants.RIGHT_SHOOTER_ID);

        m_leftShooter.setInverted(false);
        m_rightShooter.setInverted(true);

        m_rightShooter.follow((NAR_EMotor) m_leftShooter);

    }

    /**
     * Sets the Shooter State for the shooter
     * @param state Desired Shooter State
     */
    public void setState(ShooterState state) {
        this.shooterState = state;
    }

    /**
     * Begins the PID loop to achieve the desired RPM with the currently set Shooter State
     */
    public void startPID() {
        thresholdPercent = ShooterConstants.RPM_THRESHOLD_PERCENT;
        super.setSetpoint(shooterState.shooterRPM);  
        super.resetPlateauCount();
        getController().setTolerance(ShooterConstants.RPM_THRESHOLD_PERCENT * shooterState.shooterRPM);
    }

    /**
     * Begins the PID loop to achieve the desired RPM with the currently set Shooter State
     */
    public void startPID(double rpm) {
        thresholdPercent = ShooterConstants.RPM_THRESHOLD_PERCENT;
        super.setSetpoint(rpm);  
        super.resetPlateauCount();
        getController().setTolerance(ShooterConstants.RPM_THRESHOLD_PERCENT * rpm);
    }

    /**
     * Sets the state using {@link #setState(ShooterState)} and begins the PID loop to achieve it using {@link #startPID()}
     * @param state Desired Shooter State
     */
    public void beginShoot(ShooterState state) {
        Log.info("Shooter", "beginShoot state");
        Log.info("Shooter", "state: " + state.shooterRPM);
        setState(state);
        startPID();
    }

    /**
     * Sets the state using {@link #setState(ShooterState)} and begins the PID loop to achieve it using {@link #startPID()}
     * @param state Desired Shooter State
     */
    public void beginShoot(double rpm) {
        Log.info("Shooter", "beginShoot rpm");
        startPID(rpm);
    }

    public void stopShoot() {
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
        return m_leftShooter.getSelectedSensorVelocity() * ConversionConstants.FALCON_NUpS_TO_RPM;
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
        double voltageOutput = output + feedforward.calculate(setpoint); //0.003
        double voltage = RobotController.getBatteryVoltage();
        double percentOutput = voltageOutput/voltage;

        time = RobotController.getFPGATime() / 1e6;
        // if (thresholdPercent < ShooterConstants.RPM_THRESHOLD_PERCENT_MAX) {
        //     thresholdPercent += ((time - preTime) * ((ShooterConstants.RPM_THRESHOLD_PERCENT_MAX - ShooterConstants.RPM_THRESHOLD_PERCENT)) / ShooterConstants.TIME_TO_MAX_THRESHOLD);
        //     getController().setTolerance(thresholdPercent * setpoint);
        // } TODO: this thresholding could be added back

        checkPlateau(setpoint, ShooterConstants.RPM_THRESHOLD_PERCENT);

        percentOutput = MathUtil.clamp(percentOutput, -1, 1);
        percentOutput = (setpoint == 0) ? 0 : percentOutput;

        m_leftShooter.set(ControlMode.PercentOutput, percentOutput);
        //m_rightShooter.set(ControlMode.PercentOutput, -percentOutput);

        //Log.info("Shooter","percentOutput: " + percentOutput);
        //Log.info("Shooter","RPM: " + getMeasurement());
        //Log.info("Shooter", "setpoint " + setpoint);
        //Log.info("Shooter", "shooterState " + shooterState.shooterRPM);
        SmartDashboard.putNumber("Shooter Encoder", getMeasurement());

        preTime = time;
    }

    @Override
    public void simulationPeriodic() {
        m_shooterSim.setInput(
            m_leftShooter.getMotorOutputVoltage()
        );  
        m_shooterSim.update(0.02);    
        
        m_leftShooter.setSimVelocity(m_shooterSim.getAngularVelocityRadPerSec() * ShooterConstants.SHOOTER_RADIUS_METERS);
        //m_rightShooter.setQuadSimVelocity(m_shooterSim.getAngularVelocityRadPerSec() * ShooterConstants.SHOOTER_RADIUS_METERS);
    
        // SmartDashboard.putNumber("test", m_leftShooter.getMotorOutputVoltage()); 
        // SmartDashboard.putString("pogger", String.valueOf(m_shooterSim.getAngularVelocityRadPerSec()));
        SmartDashboard.putNumber("shooter RPM", m_shooterSim.getAngularVelocityRadPerSec() * 60 / (2*Math.PI));
        
    }

    public double calculateMotorVelocityFromDist(double dist) {
        return 0.00971 * Math.pow(dist, 3) - 0.289 * Math.pow(dist, 2) - 52.17 * dist + 5196;
        // if (dist < 78) {
        //     return 17.7 * dist + 2187;   
        // } else {
        //     return -4.54 * Math.pow(dist, 2) + 917 * dist - 40328;
        // }
    }
}

