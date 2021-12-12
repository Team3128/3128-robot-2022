package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import frc.team3128.Robot;
import frc.team3128.common.NAR_PIDSubsystem;
import frc.team3128.common.hardware.motor.NAR_TalonSRX;

public class Sidekick extends NAR_PIDSubsystem {

    public static enum SidekickState { // enum for the setpoint shooting state
        OFF(0),
        LONG_RANGE(4800), // long range shooting
        MID_RANGE(4080), // actual setpoint 4080 // mid range shooting
        DEFAULT(-2000), // actual default -5100
        SHORT_RANGE(2000); // short range shooting 3700

        public double sidekickRPM;

        private SidekickState(double RPM) {
            this.sidekickRPM = RPM;
        }
    }

    public static Sidekick instance;
    
    // motors
    public static NAR_TalonSRX m_sidekick; 

    // simulated motors
    public TalonSRXSimCollection m_sidekickSim;
    public FlywheelSim m_sidekickShooterSim;

    // sidekick state
    public SidekickState SIDEKICK_STATE = SidekickState.MID_RANGE;

    // feedforward (from WPILib ff)

    private final SimpleMotorFeedforward m_sidekickFeedForward =
      new SimpleMotorFeedforward(Constants.SidekickConstants.SIDEKICK_kS,
                                 Constants.SidekickConstants.SIDEKICK_kV);
    

    /**
     * Constructor for the Sidekick, initiates  
     */
    private Sidekick() {
        super(new PIDController(Constants.SidekickConstants.SIDEKICK_PID_kP, Constants.SidekickConstants.SIDEKICK_PID_kI, Constants.SidekickConstants.SIDEKICK_PID_kD));
        getController().setTolerance(2000*Constants.SidekickConstants.SIDEKICK_RPM_THRESHOLD_PERCENT, Constants.SidekickConstants.SIDEKICK_PLATEAU_THRESHOLD);


        configMotors();
        configEncoders();
        setSetpoint(0);
    }

    public static synchronized Sidekick getInstance() {
        if (instance == null) {
            instance = new Sidekick();
        }
        return instance;
    }

    /**
     * @return If the sidekick is at the desired setpoint
     */
    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    /** 
     * Configures the motors (regular+simulation) 
     */
    private void configMotors() {
        m_sidekick = new NAR_TalonSRX(Constants.SidekickConstants.SIDEKICK_ID);
        m_sidekick.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
                Constants.SidekickConstants.CAN_TIMEOUT);
        m_sidekick.setInverted(true);
        m_sidekick.setSensorPhase(true);

        if (!Robot.isReal()) {
            m_sidekickSim = new TalonSRXSimCollection(m_sidekick);

            m_sidekickShooterSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem( // plant
                    0, //kV
                    0 //kA
                ), 
                DCMotor.getVex775Pro(1), // gearbox (1 Vex775 motor)

                1/3 //gearing
            );
        }
    }

    /**
     * Configures the encoders (nothing here yet, this would be for future sidekicks with encoders)
     */

    private void configEncoders() {

    }

    /**
     * Sets the state for the sidekick
     * @param sidekickState new state for the sidekick
     */
    public void setState(SidekickState sidekickState) {
        SIDEKICK_STATE = sidekickState;
    }

    public SidekickState setState() {
        return SIDEKICK_STATE;
    }

    /**
     * Calculates and applies the voltage to reach a given setpoint using PID and feedforward
     * Converts to a percent voltage output before applying voltage
     * @param output Output from PID loop
     * @param setpoint Desired setpoint of RPM for the PID loop and FF
     */

    @Override
    protected void useOutput(double output, double setpoint) {
        //THIS IS VERY BAD - TUNE P, I, D, and feed forward later
        double voltageOutput = output + (0.0027*setpoint);//m_sidekickFeedForward.calculate(setpoint);
        double voltage = RobotController.getBatteryVoltage();

        output = voltageOutput/voltage;

        super.countPlateau(setpoint);

        output = (output > 1) ? 1 : ((output < -1) ? -1 : output);
        output = (setpoint == 0) ? 0 : output;
        
        m_sidekick.set(ControlMode.PercentOutput, output);    

        SmartDashboard.putBoolean("Sidekick isReady", isReady());
        SmartDashboard.putBoolean("Sidekick atSetpoint", atSetpoint());
        SmartDashboard.putNumber("Sidekick RPM", getMeasurement());
    }

    /**
     * @return Current RPM according to the sensor reading
     */
    @Override
    protected double getMeasurement() {
        return m_sidekick.getSelectedSensorVelocity() * Constants.ConversionConstants.SIDEKICK_ENCODER_TO_RPM;
    }

    /**
     * @return if the sidekick is ready to shoot
     */
    @Override

    public boolean isReady() {
        return true; // fix logic here
    }

    public void startPID() {
        super.startPID();
        super.setSetpoint(SIDEKICK_STATE.sidekickRPM);
        getController().setTolerance(Constants.ShooterConstants.THRESHOLD_PERCENT * SIDEKICK_STATE.sidekickRPM);
    }

    public void setPower(double power) {
        m_sidekick.set(ControlMode.PercentOutput, power);
    }

    public void beginShoot(SidekickState state) {
        setState(state);
        startPID();
    }

    public void stopShoot() {
        beginShoot(SidekickState.OFF);
    }

    @Override
    public void simulationPeriodic() {
        m_sidekickShooterSim.setInputVoltage(m_sidekick.getMotorOutputVoltage());
        m_sidekickShooterSim.update(0.02);
        m_sidekickSim.setQuadratureVelocity((int) (m_sidekickShooterSim.getAngularVelocityRadPerSec()*0.0254)); // TEMP RADIUS TO GET IT TO WORK

        SmartDashboard.putNumber("Speed", m_sidekick.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Expected Speed", m_sidekickShooterSim.getAngularVelocityRadPerSec());
    }
}
