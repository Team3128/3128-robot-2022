package frc.team3128.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3128.Constants.ClimberConstants.*;
import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.*;

import java.time.chrono.ThaiBuddhistChronology;
import java.util.ArrayList;
import java.util.List;

import frc.team3128.Constants;
import frc.team3128.Robot;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import frc.team3128.common.utility.linear_systems.PendulumSim;

/**
 * Class for the Climber Subsystem 
 */

public class Climber extends SubsystemBase {

    private static Climber instance;

    private DoubleSolenoid m_climberSolenoid;
    private NAR_TalonFX m_leftMotor, m_rightMotor;

    //Simulation
    private Timer deltaTimer;
    private PendulumSim pendulumSim;
    private boolean isConnectedToTwoRods = false;
    private boolean currentArmIsShort = false;
    private int currentLongRod = 0;

    private static List<Translation2d> rodDisplacements = new ArrayList<Translation2d>();

    //Translations in Inches
    static {
        rodDisplacements.add(new Translation2d(0, 0));
        rodDisplacements.add(new Translation2d(1.07*39.3701, 0.29*39.3701));
        rodDisplacements.add(new Translation2d(1.68*39.3701, 0.68*39.3701));
        rodDisplacements.add(new Translation2d(2.29*39.3701, 1.07*39.3701));
    }


    public Climber() {
        configMotors();
        configPneumatics();

        resetLeftEncoder();

        if(Robot.isSimulation()) {
            deltaTimer = new Timer();
            deltaTimer.start();

            pendulumSim = new PendulumSim(0, Constants.SimConstants.CLIMBER_SHORT_ARM_LENGTH/39.3701);
        }
    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    /**
     * Initializes motors and sets up CAN frame periods
     */
    private void configMotors() {
        m_leftMotor = new NAR_TalonFX(CLIMBER_MOTOR_LEFT_ID);
        m_rightMotor = new NAR_TalonFX(CLIMBER_MOTOR_RIGHT_ID);

        m_leftMotor.setInverted(true);
        // right follows left - left is the one we use for encoder/distance calculations
        m_rightMotor.follow(m_leftMotor); 
        m_rightMotor.setInverted(InvertType.OpposeMaster);
        
        m_leftMotor.setNeutralMode(NeutralMode.Brake);
        m_rightMotor.setNeutralMode(NeutralMode.Brake);

        m_leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 45);
        m_leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 45);
        m_leftMotor.setControlFramePeriod(ControlFrame.Control_3_General, 20);

        m_rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        m_rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        m_rightMotor.setControlFramePeriod(ControlFrame.Control_3_General, 45);
    }

    /**
     * Initializes pneumatics
     */
    private void configPneumatics() {
        m_climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                CLIMBER_SOLENOID_FORWARD_CHANNEL_ID, 
                                                CLIMBER_SOLENOID_BACKWARD_CHANNEL_ID);

        retractPiston();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Encoder", getCurrentTicks());
    }

    /**
     * Extends climber arms at regular power
     */
    public void bothExtend(double power) {
        m_rightMotor.set(power);
        m_leftMotor.set(power);
    }

    public void bothExtend() {
        m_rightMotor.set(CLIMBER_POWER);
        m_leftMotor.set(CLIMBER_POWER);
    }

    /**
     * Retracts/lowers climber arms at regular power
     */
    public void bothRetract(double power) {
        m_rightMotor.set(-power);
        m_leftMotor.set(-power);
    }

    public void bothRetract() {
        m_rightMotor.set(-CLIMBER_POWER);
        m_leftMotor.set(-CLIMBER_POWER);
    }

    /**
     * Extends climber arms at a slower power for resetting purposes 
     */
    public void bothManualExtend() {
        m_rightMotor.set(MANUAL_POWER);
        m_leftMotor.set(MANUAL_POWER);
    }

    /**
     * Retracts/lowers climber arms at a slower power for resetting purposes 
     */
    public void bothManualRetract() {
        m_rightMotor.set(-MANUAL_POWER);
        m_leftMotor.set(-MANUAL_POWER);
    }

    /**
     * Halts climber arms
     */
    public void bothStop() {
        m_rightMotor.set(0);
        m_leftMotor.set(0);
    }

    /**
     * Extends climber pistons to diagonal state
     */
    public void extendPiston(){
        m_climberSolenoid.set(kForward);
    }

    /**
     * Retracts climber pistons to upright state
     */
    public void retractPiston(){
        m_climberSolenoid.set(kReverse);

        if(Robot.isSimulation()) {
            this.unlatchShortArm();
        }

    }
    
    /**
     * Translates distance for climber arms to move in inches to encoder counts
     * @param distance Distance to extend/retract in inches
     * @return Corresponding encoder counts
     */
    public double getDesiredTicks(double distance) {
        return distance * (FALCON_ENCODER_RESOLUTION * CLIMBER_GEAR_RATIO) / (AXLE_DIAMETER * Math.PI);
    }

    /**
     * @return Current encoder count position of left (leader) motor
     */
    public double getCurrentTicks() {
        return m_leftMotor.getSelectedSensorPosition();
    }

    /**
     * Resets left (leader) climber encoder 
     */
    public void resetLeftEncoder() {
        m_leftMotor.setEncoderPosition(0);
    }

    /**
     * Run when the robot switches to the short arm after escalating itself towards the rod
     * Function will transfer the angle and angular velocity data from the long arm pendulum simulation
     * into the short arm.
     * 
     * Assumes that the current PendulumSim is the long arm
     */
    public void switchToShortArm() {
        pendulumSim.setState(new MatBuilder<N4,N1>(Nat.N4(), Nat.N1()).fill(
            pendulumSim.getOutput(0),
            Constants.SimConstants.CLIMBER_SHORT_ARM_LENGTH,
            pendulumSim.getOutput(2),
            0
        ));
        currentArmIsShort = true;
    }

    /**
     * Run when the long arm latches to the next rod (while the short arm is still attached to the last rod)
     * Function will set the angular velocity of the short arm pendulum to 0 to simulate the long arm stopping the
     * angular motion.
     * 
     * Assumes that the current PendulumSim is the short arm
     */
    public void latchLongArm() {
        isConnectedToTwoRods = true;
        if(currentLongRod < rodDisplacements.size()-1) {
            double xDisplacement = rodDisplacements.get(currentLongRod+1).getX()-rodDisplacements.get(currentLongRod).getX();
            double yDisplacement = rodDisplacements.get(currentLongRod+1).getY()-rodDisplacements.get(currentLongRod).getY();
            double displacement = Math.sqrt(Math.pow(Math.abs(xDisplacement), 2)+Math.pow(Math.abs(yDisplacement), 2));
            double newAngle = -1*Math.acos((displacement*displacement+Constants.SimConstants.CLIMBER_LONG_ARM_LENGTH*Constants.SimConstants.CLIMBER_LONG_ARM_LENGTH
            -Constants.SimConstants.CLIMBER_SHORT_ARM_LENGTH*Constants.SimConstants.CLIMBER_LONG_ARM_LENGTH)/(2*displacement*Constants.SimConstants.CLIMBER_LONG_ARM_LENGTH));

            // pendulumSim.setState(new MatBuilder<N4,N1>(Nat.N4(), Nat.N1()).fill(
            // newAngle, 
            // Constants.SimConstants.CLIMBER_LONG_ARM_LENGTH,
            // 0,
            // 0));

            pendulumSim = new PendulumSim(newAngle, Constants.SimConstants.CLIMBER_LONG_ARM_LENGTH/39.3701);
            System.out.println("NEW ANGLE: "+newAngle);
        }
    }

    /**
     * 
     * Run when the short arm unlatches from the previous rod, allowing the robot to rotate once again.
     */
    public void unlatchShortArm() {
        //Calculates the angle with the new rod based on the new rod's displacement from the old rod and the robot's angle with the old rod
        //If this bugs, the most obvious bug I can think of is that the angle is in radians instead of degrees
       
        //double newAngle = Math.asin(-1*(Constants.SimConstants.CLIMBER_SHORT_ARM_LENGTH*Math.sin(90-pendulumSim.getOutput(0)))/(Constants.SimConstants.CLIMBER_LONG_ARM_LENGTH));
        isConnectedToTwoRods = false;
        currentArmIsShort = false;
    }

    public void getSimAngle() {
        if(Robot.isSimulation()) {
            pendulumSim.getOutput(0);
        }
        throw new RuntimeException();
    }


    /*
     * How simulation works for climber (I made some assumptions because I could)
     * The angle on pendulumSim corresponds to the angle the long arm makes with the
     * vertical of the rod it is attached to (the "vertical of a rod" is just the line going from the rod down to the floor)
     * 
     * pendulumSim also simulates the length of the arm (based on the motor output voltage)
     * It is assumed that voltage and velocity of the arm changing length is basically linear
     * 
     * When the robot is changing between rods, the boolean isConnectedToTwoRods should be true
     * when both the short and long arms are on a rod (and the angle shouldn't change)
     * 
     * It is assumed that the short arm, the long arm, and the line between the previous and new rods form a
     * triangle when it is in the both-arms-connected-to-rods state
     * 
     * 
     */
    @Override
    public void simulationPeriodic() {


        double voltageScaleFactor = 100000; //Proportion between voltage and velocity (assuming proportional movement)
        double encoderLengthFactor = 1/1000;

        double dt = deltaTimer.get();

        //Assuming Linear Movement of the Climber Motors (proportional to voltage)

        double calculatedArmPosition = m_leftMotor.getSelectedSensorPosition()+voltageScaleFactor*m_leftMotor.getMotorOutputVoltage()*dt;
        SmartDashboard.putNumber("Maximum Arm Length", this.getDesiredTicks(Constants.SimConstants.CLIMBER_LONG_ARM_LENGTH));
        m_leftMotor.setSelectedSensorPosition(Math.min(calculatedArmPosition, this.getDesiredTicks(Constants.SimConstants.CLIMBER_LONG_ARM_LENGTH)));
        if(!currentArmIsShort) {
            pendulumSim.setInput(-1*voltageScaleFactor*encoderLengthFactor*m_leftMotor.getMotorOutputVoltage());
        }
        else {
            pendulumSim.setInput(0);
        }
        


        if(!isConnectedToTwoRods) {
            pendulumSim.update(dt);
        }

        //Should be the angle with the long arm and the corresponding rod
        double angle = pendulumSim.getOutput(0);


        SmartDashboard.putNumber("Angle of Robot with Rod", angle*(360/2*Math.PI));
        SmartDashboard.putNumber("Climber Extension Length", pendulumSim.getOutput(1));
        SmartDashboard.putNumber("Climber Left Motor",  m_leftMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("Climber Right Motor", m_rightMotor.getMotorOutputVoltage());
        SmartDashboard.putString("Climber Solenoid", m_climberSolenoid.get().toString());
        SmartDashboard.putNumber("Climber Ticks", getCurrentTicks());

        deltaTimer.reset();
    }
}