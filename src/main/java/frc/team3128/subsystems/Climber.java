package frc.team3128.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants.ClimberConstants;
import frc.team3128.Constants.ConversionConstants;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import net.thefletcher.revrobotics.enums.MotorType;

public class Climber extends SubsystemBase {
    
    public enum ClimberState {
        BOTTOM,
        TOP;
    }

    private static Climber instance;

    private ClimberState leftState, rightState;

    private DoubleSolenoid m_climberSolenoid, m_climberBreakSolenoid;
    private NAR_CANSparkMax m_leftMotor, m_rightMotor;
    private DigitalInput m_leftLimitSwitch, m_rightLimitSwitch;

    public Climber() {
        leftState = ClimberState.BOTTOM;
        rightState = ClimberState.BOTTOM;

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
        m_leftMotor = new NAR_CANSparkMax(ClimberConstants.CLIMBER_MOTOR_LEFT_ID, MotorType.kBrushless);
        m_rightMotor = new NAR_CANSparkMax(ClimberConstants.CLIMBER_MOTOR_RIGHT_ID, MotorType.kBrushless);

        m_leftMotor.setIdleMode(ClimberConstants.CLIMBER_NEUTRAL_MODE);
        m_rightMotor.setIdleMode(ClimberConstants.CLIMBER_NEUTRAL_MODE);
    }

    private void configSensors() {
        m_leftLimitSwitch = new DigitalInput(ClimberConstants.CLIMBER_SENSOR_LEFT_ID);
        m_rightLimitSwitch = new DigitalInput(ClimberConstants.CLIMBER_SENSOR_RIGHT_ID);
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
        if (getLeftState() == ClimberState.BOTTOM && getLeftSwitch() && getCurrentTicksLeft() > (getDesiredTicks(ClimberConstants.CLIMBER_HEIGHT/2))) {
            setLeftState(ClimberState.TOP);
        }
        else if (getLeftState() == ClimberState.TOP && getLeftSwitch() && getCurrentTicksLeft() < (getDesiredTicks(ClimberConstants.CLIMBER_HEIGHT/2))){
            setLeftState(ClimberState.BOTTOM);
            resetLeftEncoder();
        }

        if (getRightState() == ClimberState.BOTTOM && getRightSwitch() && getCurrentTicksRight() > (getDesiredTicks(ClimberConstants.CLIMBER_HEIGHT/2))) {
                setLeftState(ClimberState.TOP);
        }
        else if (getRightState() == ClimberState.TOP && getRightSwitch() && getCurrentTicksRight() < (getDesiredTicks(ClimberConstants.CLIMBER_HEIGHT/2))){
            setLeftState(ClimberState.BOTTOM);
            resetRightEncoder();
        }

        SmartDashboard.putString("Climber L state", leftState.toString());
        SmartDashboard.putString("Climber R state", rightState.toString());

    }
    
    public void leftRetract(){
        if (getLeftState() == ClimberState.TOP){
            m_leftMotor.set(-ClimberConstants.CLIMBER_POWER);
        }
    }

    public void rightRetract(){
        if (getLeftState() == ClimberState.TOP){
            m_rightMotor.set(-ClimberConstants.CLIMBER_POWER);
        }
    }

    public void leftExtend(){
        if (getLeftState() == ClimberState.BOTTOM){
            m_leftMotor.set(ClimberConstants.CLIMBER_POWER);
        }
    }

    public void rightExtend(){
        if (getRightState() == ClimberState.BOTTOM){
            m_rightMotor.set(ClimberConstants.CLIMBER_POWER);
        }
    }

    public void extendBoth() {
        m_rightMotor.set(ClimberConstants.CLIMBER_POWER);
        m_leftMotor.set(ClimberConstants.CLIMBER_POWER);
    }

    public void retractBoth() {
        m_rightMotor.set(-ClimberConstants.CLIMBER_POWER);
        m_leftMotor.set(-ClimberConstants.CLIMBER_POWER);
    }

    public void stopBoth() {
        m_rightMotor.set(0);
        m_leftMotor.set(0);
    }

    public void leftStop(){
        m_leftMotor.set(0);
    }

    public void rightStop(){
        m_rightMotor.set(0);
    }

    public void extendPiston(){
        m_climberSolenoid.set(kForward);
    }

    public void retractPiston(){
        m_climberSolenoid.set(kReverse);
    }

    public void engageBreak() {
        m_climberBreakSolenoid.set(kForward);
    }

    public void disengageBreak() {
        m_climberBreakSolenoid.set(kReverse);
    }

    public void setLeftState(ClimberState state) {
        leftState = state;
    }
    public void setRightState(ClimberState state) {
        rightState = state;
    }

    public boolean getLeftSwitch() {
        return !m_leftLimitSwitch.get();
    }
    
    public boolean getRightSwitch() {
        return !m_rightLimitSwitch.get();
    }
    
    public double getDesiredTicks(double distance) {
        double desiredTicks = distance * (((ConversionConstants.SPARK_ENCODER_RESOLUTION)*(ClimberConstants.CLIMBER_GEAR_RATIO)) / ((ClimberConstants.AXLE_DIAMETER)*Math.PI));
        return desiredTicks;
    }

    public double getCurrentTicksLeft() {
        return m_leftMotor.getSelectedSensorPosition();
    }
    
    public double getCurrentTicksRight() {
        return m_rightMotor.getSelectedSensorPosition();
    }

    public ClimberState getLeftState() {
        return leftState;
    }
    
    public ClimberState getRightState() {
        return rightState;
    } 

    public void resetLeftEncoder() {
        m_rightMotor.setEncoderPosition(0);
    }
    public void resetRightEncoder() {
        m_rightMotor.setEncoderPosition(0);
    }

}