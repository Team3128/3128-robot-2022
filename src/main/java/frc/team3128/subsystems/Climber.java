package frc.team3128.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team3128.Constants;
import frc.team3128.Constants.ClimberConstants;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.infrastructure.NAR_EMotor;

import net.thefletcher.revrobotics.enums.MotorType;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DigitalInput;

public class Climber extends SubsystemBase {
    public enum ClimberState {
        BOTTOM,
        TOP;
    }

    private static Climber instance;
    private ClimberState climberLeftState;
    private ClimberState climberRightState;
    private DoubleSolenoid m_climberSolenoid, m_climberBreakSolenoid;
    private NAR_CANSparkMax m_climbMotorL, m_climbMotorR;
    private DigitalInput climberLimitSwitchL, climberLimitSwitchR;



    public Climber() {
        climberLeftState = ClimberState.BOTTOM;
        climberRightState = ClimberState.BOTTOM;
        configMotors();
        configSensors();
        configPneumatics();
        resetLeftEncoder();
        resetRightEncoder();
    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    private void configMotors() {
        m_climbMotorL = new NAR_CANSparkMax(Constants.ClimberConstants.CLIMBER_MOTOR_1_ID, MotorType.kBrushless);
        m_climbMotorR = new NAR_CANSparkMax(Constants.ClimberConstants.CLIMBER_MOTOR_2_ID, MotorType.kBrushless);

        m_climbMotorL.setIdleMode(Constants.ClimberConstants.CLIMBER_NEUTRAL_MODE);
        m_climbMotorR.setIdleMode(Constants.ClimberConstants.CLIMBER_NEUTRAL_MODE);
    }

    private void configSensors() {
        climberLimitSwitchL = new DigitalInput(Constants.ClimberConstants.CLIMBER_SENSOR_1_ID);
        climberLimitSwitchR = new DigitalInput(Constants.ClimberConstants.CLIMBER_SENSOR_2_ID);
    }
    
    private void configPneumatics() {
        m_climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
                                               ClimberConstants.CLIMBER_SOLENOID_FORWARD_CHANNEL_ID, 
                                               ClimberConstants.CLIMBER_SOLENOID_BACKWARD_CHANNEL_ID);
        m_climberBreakSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
                                                    ClimberConstants.CLIMBER_SOLENOID_BREAK_FORWARD_CHANNEL_ID, 
                                                    ClimberConstants.CLIMBER_SOLENOID_BREAK_BACKWARD_CHANNEL_ID);
    }

    @Override
    public void periodic() {
        if (getClimberLeftState() == ClimberState.BOTTOM 
            && getLeftSwitch() 
            && getCurrentTicksLeft() > (getDesiredTicks(Constants.ClimberConstants.CLIMBER_HEIGHT/2))) {

            setClimberLeftState(ClimberState.TOP);
        }
        else if (getClimberLeftState() == ClimberState.TOP 
                 && getLeftSwitch() 
                 && getCurrentTicksLeft() < (getDesiredTicks(Constants.ClimberConstants.CLIMBER_HEIGHT/2))) {
            
            setClimberLeftState(ClimberState.BOTTOM);
            resetLeftEncoder();
        }

        if (getClimberRightState() == ClimberState.BOTTOM 
            && getRightSwitch() 
            && getCurrentTicksRight() > (getDesiredTicks(Constants.ClimberConstants.CLIMBER_HEIGHT/2))) {
            
            setClimberLeftState(ClimberState.TOP);
        }
        else if (getClimberRightState() == ClimberState.TOP 
                && getRightSwitch() 
                && getCurrentTicksRight() < (getDesiredTicks(Constants.ClimberConstants.CLIMBER_HEIGHT/2))) {

            setClimberLeftState(ClimberState.BOTTOM);
            resetRightEncoder();
        }
    }
    
    public void climberLeftRetract(){
        if (getClimberLeftState() == ClimberState.TOP){
            m_climbMotorL.set(-Constants.ClimberConstants.CLIMBER_POWER);
        }
    }

    public void climberRightRetract(){
        if (getClimberLeftState() == ClimberState.TOP){
            m_climbMotorR.set(-Constants.ClimberConstants.CLIMBER_POWER);
        }
    }

    public void climberLeftExtend(){
        if (getClimberLeftState() == ClimberState.BOTTOM){
            m_climbMotorL.set(Constants.ClimberConstants.CLIMBER_POWER);
        }
    }

    public void climberRightExtend(){
        if (getClimberRightState() == ClimberState.BOTTOM){
            m_climbMotorR.set(Constants.ClimberConstants.CLIMBER_POWER);
        }
    }

    public void climberLeftStop(){
        m_climbMotorL.set(0);
    }

    public void climberRightStop(){
        m_climbMotorR.set(0);
    }

    public void extendArm(){
        m_climberSolenoid.set(kForward);
    }

    public void retractArm(){
        m_climberSolenoid.set(kReverse);
    }

    public void breakClimb() {
        m_climberBreakSolenoid.set(kForward);
    }

    public void unbreakClimb() {
        m_climberBreakSolenoid.set(kReverse);
    }

    public void setClimberLeftState(ClimberState state) {
        climberLeftState = state;
    }
    public void setClimberRightState(ClimberState state) {
        climberRightState = state;
    }

    public boolean getLeftSwitch() {
        return climberLimitSwitchL.get();
    }
    
    public boolean getRightSwitch() {
        return climberLimitSwitchR.get();
    }
    
    public double getDesiredTicks(double distance) {
        double desiredTicks = distance * (((Constants.ConversionConstants.SPARK_ENCODER_RESOLUTION)
                                       * (Constants.ClimberConstants.CLIMBER_GEAR_RATIO)) 
                                       / ((Constants.ClimberConstants.AXLE_DIAMETER)*Math.PI));
        return desiredTicks;
    }

    public double getCurrentTicksLeft() {
        return m_climbMotorL.getSelectedSensorPosition();
    }
    
    public double getCurrentTicksRight() {
        return m_climbMotorR.getSelectedSensorPosition();
    }

    public ClimberState getClimberLeftState() {
        return climberLeftState;
    }
    
    public ClimberState getClimberRightState() {
        return climberRightState;
    } 

    public void resetLeftEncoder() {
        m_climbMotorR.setEncoderPosition(0);
    }
    public void resetRightEncoder() {
        m_climbMotorR.setEncoderPosition(0);
    }

}