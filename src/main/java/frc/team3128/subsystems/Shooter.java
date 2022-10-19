package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import static frc.team3128.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import frc.team3128.ConstantsInt;
import frc.team3128.commands.CmdShootAlign;

import static frc.team3128.Constants.ConversionConstants.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.common.utility.interpolation.InterpolatingDouble;

/**
 * Class for the Shooter Subsystem 
 */

public class Shooter extends PIDSubsystem {

    private static Shooter instance;

    //Motors
    private NAR_TalonFX m_leftShooter;
    private NAR_TalonFX m_rightShooter;

    private int plateauCount = 0;

    public DoubleSupplier m_ff;
    public DoubleSupplier m_setpoint;

    private FlywheelSim m_shooterSim;

    public Shooter() {
        super(new PIDController(kP, kI, kD));

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

    /**
     * Initializes motors and sets up CAN frame periods
     */
    private void configMotors() {
        m_leftShooter = new NAR_TalonFX(LEFT_SHOOTER_ID);
        m_rightShooter = new NAR_TalonFX(RIGHT_SHOOTER_ID);

        m_leftShooter.setInverted(false);
        m_rightShooter.setInverted(true);

        m_rightShooter.follow(m_leftShooter);

        m_leftShooter.setNeutralMode(NeutralMode.Coast);
        m_rightShooter.setNeutralMode(NeutralMode.Coast);

        m_rightShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        m_rightShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        m_rightShooter.setControlFramePeriod(ControlFrame.Control_3_General, 45);
        
        m_leftShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 15);
        m_leftShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 15);
        m_leftShooter.setControlFramePeriod(ControlFrame.Control_3_General, 20);

        m_leftShooter.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 30, 0.1));
    }

    public void initShuffleboard() {
        // General Tab
        NAR_Shuffleboard.addData("General","Shooter Setpoint",this::getSetpoint).withPosition(5, 2);
        NAR_Shuffleboard.addData("General","Shooter RPM",this::getMeasurement).withPosition(4, 2);
        NAR_Shuffleboard.addData("General","Shooter isReady",this::isReady).withPosition(2, 3).withSize(2, 1);
        // Shooter Tab
        NAR_Shuffleboard.addData("Shooter + Hood","Shooter Setpoint",this::getSetpoint).withPosition(2, 0);
        NAR_Shuffleboard.addData("Shooter + Hood","Shooter RPM",this::getMeasurement).withPosition(3, 0);
        NAR_Shuffleboard.addData("Shooter + Hood","Shooter isReady",this::isReady).withSize(2, 1).withPosition(0, 2);
        NAR_Shuffleboard.addComplex("Shooter + Hood","Shooter", this).withPosition(0, 0);
        NAR_Shuffleboard.addComplex("Shooter + Hood", "Shooter_PID",m_controller).withPosition(2,1).withSize(2,2);
        m_ff = NAR_Shuffleboard.debug("Shooter + Hood","Shooter FF",kF,4,1);
        if(RobotBase.isSimulation()) {
            NAR_Shuffleboard.addData("Shooter + Hood","Sim Shooter RPM", ()-> (m_shooterSim.getAngularVelocityRadPerSec() * 60 / (2*Math.PI)));
        }
        m_setpoint = NAR_Shuffleboard.debug("Shooter + Hood","SET_RPM",0,4,2);

    }

    @Override
    public void periodic() {
        // setSetpoint(m_setpoint.getAsDouble());
        NAR_Shuffleboard.put("Shooter + Hood","Pred RPM", calculateRPMFromDist(LimelightSubsystem.getInstance().calculateShooterDistance()),4,0);
        super.periodic();
    }

    /**
     * Begins the PID loop to achieve the desired RPM to shoot
     */
    public void beginShoot(double rpm) {
        // rpm = m_setpoint.getAsDouble(); // uncomment for interpolation
        enable();
        setSetpoint(rpm);
        getController().setTolerance(RPM_THRESHOLD_PERCENT * rpm);
    }

    public void reverseShoot() {
        disable();
        m_leftShooter.set(ControlMode.PercentOutput, -0.3);
    }

    /**
     * Stops the shooter from shooting by setting the setpoint to 0
     */
    public void stopShoot() {
        plateauCount = 0;
        setSetpoint(0);
        disable();
    }

    /**
     * Gets the current shooter RPM 
     */
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
        double ff = m_ff.getAsDouble() * setpoint;
        double voltageOutput = output + ff;

        if (getController().atSetpoint() && (setpoint != 0)) {
            plateauCount ++;
        } else {
            plateauCount = 0;
        }

        if (setpoint == 0) voltageOutput = 0;
            
        m_leftShooter.set(MathUtil.clamp(voltageOutput / 12.0, -1, 1));
    }

    /**
     * Returns if the shooter is ready to shoot
     */
    public boolean isReady() {
        return (plateauCount >= PLATEAU_COUNT) && (getSetpoint() != 0);
    }

    /**
     * Resets the plateau count for Shooter.isReady()
     */
    public void resetPlateauCount() {
        plateauCount = 0;
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
        // SmartDashboard.putNumber("shooter RPM", m_shooterSim.getAngularVelocityRadPerSec() * 60 / (2*Math.PI));
        
    }

    /**
     * Calculates the preferred shooter RPM for shooting at distance dist
     * Uses InterpolatingTreeMap in Constants
     */
    public double calculateRPMFromDist(double dist) {
        return shooterSpeedsMap.getInterpolated(new InterpolatingDouble(dist)).value;
    }
}

