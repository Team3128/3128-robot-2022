package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import frc.team3128.Robot;
import frc.team3128.common.NAR_PIDSubsystem;
import frc.team3128.hardware.*;

public class Shooter extends NAR_PIDSubsystem {

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
    private BaseTalon m_leftShooter, m_rightShooter;
    //Simulated Motors
    private TalonSRXSimCollection m_leftShooterSim;

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
     * Creates the PID Controller and Instantiates the Motors (with simulated counterparts if necessary)
     */
    public Shooter() {
        
        super(new PIDController(Constants.ShooterConstants.SHOOTER_PID_kP, Constants.ShooterConstants.SHOOTER_PID_kI, Constants.ShooterConstants.SHOOTER_PID_kD), Constants.ShooterConstants.PLATEAU_COUNT);

        //Robot is real
        if(Robot.isReal()) {
            m_leftShooter = new NAR_TalonFX(Constants.ShooterConstants.LEFT_SHOOTER_ID);
            m_rightShooter = new NAR_TalonFX(Constants.ShooterConstants.RIGHT_SHOOTER_ID);
        }
        else {
            //Robot is a simulation
            m_leftShooter = new WPI_TalonSRX(Constants.ShooterConstants.LEFT_SHOOTER_ID);
            m_rightShooter = new WPI_TalonSRX(Constants.ShooterConstants.RIGHT_SHOOTER_ID);
            m_leftShooterSim = new TalonSRXSimCollection(m_leftShooter);
            m_shooterSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(
                    0, //kV
                    0 //kA
                ),
                DCMotor.getFalcon500(2), //gearbox
                1.5 //gearing
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
        return m_leftShooter.getSelectedSensorVelocity(0) * Constants.ConversionConstants.ENCODER_TO_RPM;
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
        double voltageOutput = output + (setpoint * 0.0019); // m_shooterFeedforward.calculate(setpoint);
        double voltage = RobotController.getBatteryVoltage();
        double percentOutput = voltageOutput/voltage;

        time = RobotController.getFPGATime() / 1e6;
        if (thresholdPercent < Constants.ShooterConstants.RPM_THRESHOLD_PERCENT_MAX) {
            thresholdPercent += ((time - preTime) * ((Constants.ShooterConstants.RPM_THRESHOLD_PERCENT_MAX - Constants.ShooterConstants.RPM_THRESHOLD_PERCENT)) / Constants.ShooterConstants.TIME_TO_MAX_THRESHOLD);
            getController().setTolerance(thresholdPercent * setpoint);
        }

        super.useOutput(setpoint);

        preTime = time;


        percentOutput = (percentOutput > 1) ? 1 : ((percentOutput < -1) ? -1 : percentOutput);
        percentOutput = (setpoint == 0) ? 0 : percentOutput;

        m_leftShooter.set(ControlMode.PercentOutput, percentOutput);
        m_rightShooter.set(ControlMode.PercentOutput, -percentOutput);
    }

    @Override
    public void simulationPeriodic() {
        m_shooterSim.setInputVoltage(
            m_leftShooter.getMotorOutputVoltage()
        );  
        m_shooterSim.update(0.02);    
        
        m_leftShooterSim.setQuadratureVelocity((int) (m_shooterSim.getAngularVelocityRadPerSec() * 0.0254));

        SmartDashboard.putNumber("Speed", m_leftShooter.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Expected Speed", m_shooterSim.getAngularVelocityRadPerSec());

    }
    
}
