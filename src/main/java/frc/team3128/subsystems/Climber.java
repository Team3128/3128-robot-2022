package frc.team3128.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team3128.Constants;
import frc.team3128.Constants.ClimberConstants;
import frc.team3128.common.hardware.motor.NAR_CANSparkMax;
import frc.team3128.common.infrastructure.NAR_EMotor;

import net.thefletcher.revrobotics.enums.MotorType;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DigitalInput;

public class Climber extends SubsystemBase {
    public enum ClimberState {
        BOTTOM(0),
        TOP(Constants.ClimberConstants.CLIMBER_HEIGHT);

        public double climberHeight;
        private ClimberState(double climberHeight){
            this.climberHeight = climberHeight;
        }
    }

    private static Climber instance;
    private ClimberState climberState;
    private DoubleSolenoid m_solenoid;
    private NAR_CANSparkMax m_climbMotor1, m_climbMotor2;
    DigitalInput climberLimitSwitch;



    public Climber() {
        climberState = ClimberState.BOTTOM;
        configMotors();
        configSensors();
        configPneumatics();
    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    private void configMotors() {
        m_climbMotor1 = new NAR_CANSparkMax(Constants.ClimberConstants.CLIMBER_MOTOR_1_ID, MotorType.kBrushless);
        m_climbMotor2 = new NAR_CANSparkMax(Constants.ClimberConstants.CLIMBER_MOTOR_2_ID, MotorType.kBrushless);

        m_climbMotor2.follow((NAR_EMotor)m_climbMotor1);
        //m_climbMotor1.setNeutralMode(Constants.ClimberConstants.CLIMBER_NEUTRAL_MODE);
        m_solenoid.set(kOff);  
    }

    private void configSensors() {
        climberLimitSwitch = new DigitalInput(Constants.ClimberConstants.CLIMBER_SENSOR_ID);
    }
    
    private void configPneumatics() {
        m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.CLIMBER_SOLENOID_FORWARD_CHANNEL_ID, ClimberConstants.CLIMBER_SOLENOID_BACKWARD_CHANNEL_ID);
    }

    @Override
    public void periodic() {
        if (getClimberState() == ClimberState.BOTTOM && getSwitch() && getCurrentTicks() > (getDesiredTicks(Constants.ClimberConstants.CLIMBER_HEIGHT/2))) {
            setClimberState(ClimberState.TOP);
        }
        else if (getClimberState() == ClimberState.TOP && getSwitch() && getCurrentTicks() < (getDesiredTicks(Constants.ClimberConstants.CLIMBER_HEIGHT/2))){
            setClimberState(ClimberState.BOTTOM);
            resetEncoder();
        }
    }
    
    public void climberRetract(){
        if (getClimberState() == ClimberState.TOP){
            m_climbMotor1.set(Constants.ClimberConstants.CLIMBER_POWER);
        }
    }

    public void climberExtend(){
        if (getClimberState() == ClimberState.BOTTOM){
            m_climbMotor1.set(Constants.ClimberConstants.CLIMBER_POWER);
        }
    }

    public void climberStop(){
        m_climbMotor1.set(0);
    }

    public void extendArm(){
        m_solenoid.set(kForward);
    }

    public void retractArm(){
        m_solenoid.set(kReverse);
    }

    public void setClimberState(ClimberState state) {
        climberState = state;
    }

    public boolean getSwitch() {
        return climberLimitSwitch.get();
    }
    
    public double getDesiredTicks(double distance) {
        double desiredTicks = distance * (((Constants.ConversionConstants.SPARK_ENCODER_RESOLUTION)*(Constants.ClimberConstants.CLIMBER_GEAR_RATIO)) / ((Constants.ClimberConstants.AXLE_DIAMETER)*Math.PI));
        return desiredTicks;
    }

    public double getCurrentTicks() {
        return m_climbMotor1.getSelectedSensorPosition();
    }

    public ClimberState getClimberState() {
        return climberState;
    } 

    public void resetEncoder() {
        m_climbMotor1.setEncoderPosition(0);
    }

}