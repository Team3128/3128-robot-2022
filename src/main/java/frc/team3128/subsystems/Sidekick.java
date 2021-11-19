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
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.team3128.Robot;
import frc.team3128.hardware.AN_TalonSRX;

public class Sidekick extends PIDSubsystem {

    public static enum SidekickState {
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

    public static final Sidekick instance = new Sidekick();
    public static AN_TalonSRX SIDEKICK; 
    public TalonSRXSimCollection m_sidekickSim;
    public FlywheelSim m_sidekickShooterSim;

    public SidekickState SIDEKICK_STATE = SidekickState.MID_RANGE;

    private final SimpleMotorFeedforward m_sidekickFeedForward =
      new SimpleMotorFeedforward(Constants.SidekickConstants.SIDEKICK_kS,
                                 Constants.SidekickConstants.SIDEKICK_kV);
    

    private Sidekick() {
        super(new PIDController(Constants.SidekickConstants.SIDEKICK_PID_kP, Constants.SidekickConstants.SIDEKICK_PID_kI, Constants.SidekickConstants.SIDEKICK_PID_kD));
        getController().setTolerance(2000*Constants.SidekickConstants.SIDEKICK_RPM_THRESHOLD_PERCENT);

        configMotors();
        configEncoders();
        setSetpoint(0);
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    private void configMotors() {
        SIDEKICK = new AN_TalonSRX(Constants.SidekickConstants.SIDEKICK_ID);
        SIDEKICK.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
                Constants.SidekickConstants.CAN_TIMEOUT);
        SIDEKICK.setInverted(true);
        SIDEKICK.setSensorPhase(true);

        if (!Robot.isReal()) {
            m_sidekickSim = new TalonSRXSimCollection(SIDEKICK);
            m_sidekickShooterSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem( // plant
                    0, //kV
                    0 //kA
                ), 
                DCMotor.getVex775Pro(1),
                1/3 //gearing
            );
        }
    }

    private void configEncoders() {

    }

    public static Sidekick getInstance() {
        return instance;
    }

    public void setState(SidekickState sidekickState) {
        SIDEKICK_STATE = sidekickState;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double voltageOutput = output + m_sidekickFeedForward.calculate(setpoint);
        double voltage = RobotController.getBatteryVoltage();

        output = voltageOutput/voltage;

        output = (output > 1 ) ? 1 : ((output < -1) ? -1 : output);
        output = (setpoint == 0) ? 0 : output;
        
        SIDEKICK.set(ControlMode.PercentOutput, output);    
    }

    @Override
    protected double getMeasurement() {
        return SIDEKICK.getSelectedSensorVelocity() * Constants.ConversionConstants.SIDEKICK_ENCODER_TO_RPM;
    }

    public boolean isReady() {
        return true; // fix logic here
    }

    public void setPower(double power) {
        SIDEKICK.set(ControlMode.PercentOutput, power);
    }

    public void shoot() {
        setSetpoint(SIDEKICK_STATE.sidekickRPM);
    }

    public void counterShoot() {
        setSetpoint(0);
    }

    @Override
    public void simulationPeriodic() {
        m_sidekickShooterSim.setInputVoltage(SIDEKICK.getMotorOutputVoltage());
        m_sidekickShooterSim.update(0.02);
        m_sidekickSim.setQuadratureVelocity((int) (m_sidekickShooterSim.getAngularVelocityRadPerSec()*0.0254)); // TEMP RADIUS TO GET IT TO WORK

        SmartDashboard.putNumber("Speed", SIDEKICK.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Expected Speed", m_sidekickShooterSim.getAngularVelocityRadPerSec());
    }
}
