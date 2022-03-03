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

        OFF(0, 0, 0, 0, 0, 0, 0),
        UPPERHUB(0, ShooterConstants.HIGH_kP, ShooterConstants.HIGH_kI, ShooterConstants.HIGH_kD, ShooterConstants.HIGH_kS, ShooterConstants.HIGH_kV, ShooterConstants.HIGH_kA),
        LOWERHUB(1250, ShooterConstants.LOW_kP, ShooterConstants.LOW_kI, ShooterConstants.LOW_kD, ShooterConstants.LOW_kS, ShooterConstants.LOW_kV, ShooterConstants.LOW_kA);

        public double shooterRPM;
        public double kP;
        public double kI;
        public double kD;

        public double kS;
        public double kV;
        public double kA;

        private ShooterState(double RPM, double kP, double kI, double kD, double kS, double kV, double kA) {
            this.shooterRPM = RPM;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;

            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
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

    private SimpleMotorFeedforward lowFF = new SimpleMotorFeedforward(ShooterConstants.LOW_kS, ShooterConstants.LOW_kV, ShooterConstants.LOW_kA);
    private SimpleMotorFeedforward highFF = new SimpleMotorFeedforward(ShooterConstants.HIGH_kS, ShooterConstants.HIGH_kV, ShooterConstants.HIGH_kA);


    public Shooter() {
        super(new PIDController(ShooterConstants.HIGH_kP, ShooterConstants.HIGH_kI, ShooterConstants.HIGH_kD), ShooterConstants.PLATEAU_COUNT);
    
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
        // super.resetPlateauCount();
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
        double ff = 0;
        if (shooterState == ShooterState.LOWERHUB)
            ff = lowFF.calculate(setpoint);
        else if (shooterState == ShooterState.UPPERHUB)
            ff = highFF.calculate(setpoint);
        double voltageOutput = output + ff;
        double voltage = RobotController.getBatteryVoltage();
        double percentOutput = voltageOutput / voltage;

        time = RobotController.getFPGATime() / 1e6;
        if (thresholdPercent < ShooterConstants.RPM_THRESHOLD_PERCENT_MAX) {
            thresholdPercent += (time - preTime) * (ShooterConstants.RPM_THRESHOLD_PERCENT_MAX - ShooterConstants.RPM_THRESHOLD_PERCENT) / ShooterConstants.TIME_TO_MAX_THRESHOLD;
            getController().setTolerance(thresholdPercent * setpoint);
        }

        checkPlateau(setpoint, thresholdPercent);

        percentOutput = MathUtil.clamp(percentOutput, -1, 1);
        percentOutput = (setpoint == 0) ? 0 : percentOutput;

        m_leftShooter.set(ControlMode.PercentOutput, percentOutput);
        //m_rightShooter.set(ControlMode.PercentOutput, -percentOutput);

        //Log.info("Shooter","percentOutput: " + percentOutput);
        //Log.info("Shooter","RPM: " + getMeasurement());
        //Log.info("Shooter", "setpoint " + setpoint);
        //Log.info("Shooter", "shooterState " + shooterState.shooterRPM);
        // SmartDashboard.putNumber("Shooter Encoder", getMeasurement());

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

        // dist += (13 - Robot.voltageRollingAvg) * 7;

        return 0.00971 * Math.pow(dist, 3) - 0.289 * Math.pow(dist, 2) - 52.17 * dist + 5196 + 600;
        // double rpm;
        // if (dist < 78) {
        //     rpm = 17.7 * dist + 2187;   
        // } else {
        //     rpm = -4.54 * Math.pow(dist, 2) + 917 * dist - 40328;
        // }
        // return rpm + 1000;
    }
}

