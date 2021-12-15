package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3128.Robot;
import frc.team3128.common.NAR_EMotor;
import frc.team3128.common.NAR_PIDSubsystem;
import frc.team3128.common.hardware.motor.NAR_TalonFX;

public class Shooter extends NAR_PIDSubsystem {

    /**
     * Different possible RPM setpoints
     */
    public enum ShooterState {
        OFF(0),
        LONG_RANGE(6000), // long range shooting
        MID_RANGE(4800), //4800 actual setpoint  // mid range shooting
        SHORT_RANGE(3500),
        GREEN(1200),
        YELLOW(5000),
        BLUE(3330),
        RED(3333); // short range shooting 3700

        public double shooterRPM;

        private ShooterState(double RPM) {
            this.shooterRPM = RPM;
        }
    }

    private static Shooter instance;

    //Motors
    private NAR_TalonFX m_leftShooter = new NAR_TalonFX(Constants.ShooterConstants.LEFT_SHOOTER_ID);
    private NAR_TalonFX m_rightShooter = new NAR_TalonFX(Constants.ShooterConstants.RIGHT_SHOOTER_ID);

    //Simulated Shooter
    private FlywheelSim m_shooterSim;

    //Feed Forward
    private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(Constants.ShooterConstants.SHOOTER_KS,
                                 Constants.ShooterConstants.SHOOTER_KV,
                                 Constants.ShooterConstants.SHOOTER_KA);

    //Shooter State
    private ShooterState shooterState = ShooterState.OFF;

    //Threshold Information
    private double time = 0, preTime = 0;
    private double thresholdPercent = Constants.ShooterConstants.RPM_THRESHOLD_PERCENT;
    

    /**
     * Creates the PID Controller (with sim if necessary)
     */
    public Shooter() {
        
        super(new PIDController(Constants.ShooterConstants.SHOOTER_PID_kP, 
                                Constants.ShooterConstants.SHOOTER_PID_kI, 
                                Constants.ShooterConstants.SHOOTER_PID_kD), 
                                Constants.ShooterConstants.PLATEAU_COUNT);
    
        //Robot is a simulation
        if(Robot.isSimulation()){
            m_shooterSim = new FlywheelSim(
                Constants.ShooterConstants.SHOOTER_CHAR,
                Constants.ShooterConstants.SHOOTER_GEARBOX,
                Constants.ShooterConstants.SHOOTER_GEARING 
            );
        }
    }

    public static synchronized Shooter getInstance() {
        if(instance == null) {
            instance = new Shooter();
        }
        return instance;
    }


    /**
     * @return If the shooter is at the setpoint RPM
     */
    public boolean atSetpoint() {
        return m_controller.atSetpoint();
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
        super.startPID();
        super.setSetpoint(shooterState.shooterRPM);
        getController().setTolerance(Constants.ShooterConstants.THRESHOLD_PERCENT * shooterState.shooterRPM);
    }

    /**
     * Sets the state using {@link #setState(ShooterState)} and begins the PID loop to achieve it using {@link #startPID()}
     * @param state Desired Shooter State
     */
    public void beginShoot(ShooterState state) {
        setState(state);
        startPID();
    }

    public void stopShoot() {
        beginShoot(ShooterState.OFF);
    }

    /**
     * @return The Shooter State
     */
    public ShooterState getState() {
        return shooterState;
    }

    @Override
    protected double getMeasurement() {
        return m_leftShooter.getSelectedSensorVelocity() * Constants.ConversionConstants.ENCODER_TO_RPM;
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
        double voltageOutput = output + m_shooterFeedforward.calculate(setpoint);
        double voltage = RobotController.getBatteryVoltage();
        double percentOutput = voltageOutput/voltage;

        time = RobotController.getFPGATime() / 1e6;
        // if (thresholdPercent < Constants.ShooterConstants.RPM_THRESHOLD_PERCENT_MAX) {
        //     thresholdPercent += ((time - preTime) * ((Constants.ShooterConstants.RPM_THRESHOLD_PERCENT_MAX - Constants.ShooterConstants.RPM_THRESHOLD_PERCENT)) / Constants.ShooterConstants.TIME_TO_MAX_THRESHOLD);
        //     getController().setTolerance(thresholdPercent * setpoint);
        // }

        preTime = time;


        percentOutput = (percentOutput > 1) ? 1 : ((percentOutput < -1) ? -1 : percentOutput);
        percentOutput = (setpoint == 0) ? 0 : percentOutput;

        m_leftShooter.set(ControlMode.PercentOutput, percentOutput);
        m_rightShooter.set(ControlMode.PercentOutput, -percentOutput);

        SmartDashboard.putNumber("Shooter RPM", getMeasurement());
        SmartDashboard.putBoolean("Shooter isReady", isReady());
        SmartDashboard.putBoolean("Shooter atSetpoint", atSetpoint());
        SmartDashboard.putNumber("Shooter setpoint", setpoint);
        SmartDashboard.putNumber("Shooter threshold percent", thresholdPercent);
        SmartDashboard.putNumber("Shooter plateau", plateauCount);

        super.countPlateau(setpoint);

        SmartDashboard.putBoolean("Shooter atSetpoint2", atSetpoint());
        SmartDashboard.putBoolean("Shooter atSetpoint3", m_controller.atSetpoint());
        SmartDashboard.putBoolean("NarSet atSetpoint4", super.narSet());
        SmartDashboard.putBoolean("GetController atSetpoint4", getController().atSetpoint());
        
    }

    @Override
    public void simulationPeriodic() {
        m_shooterSim.setInput(
            m_leftShooter.getMotorOutputVoltage()
            //m_rightShooter.getMotorOutputVoltage()
        );  
        m_shooterSim.update(0.02);    
        
        m_leftShooter.setQuadSimVelocity(m_shooterSim.getAngularVelocityRadPerSec() * Constants.ShooterConstants.SHOOTER_RADIUS_METERS);
        //m_rightShooter.setQuadSimVelocity(m_shooterSim.getAngularVelocityRadPerSec() * Constants.ShooterConstants.SHOOTER_RADIUS_METERS);

        SmartDashboard.putNumber("test", m_leftShooter.getMotorOutputVoltage()); 
        SmartDashboard.putNumber("Expected Shooter Speed (rpm)", m_shooterSim.getAngularVelocityRadPerSec()); //* 60 / (2*Math.PI) );
        SmartDashboard.putString("pogger", String.valueOf(m_shooterSim.getAngularVelocityRadPerSec()));
    }
    
    public void setMotorVelocity(double rpm) {
        m_leftShooter.set(ControlMode.Velocity, rpm / Constants.ConversionConstants.ENCODER_TO_RPM);
        m_rightShooter.set(ControlMode.Velocity, -rpm / Constants.ConversionConstants.ENCODER_TO_RPM);
    }
}
