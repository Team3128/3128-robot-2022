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
import frc.team3128.common.infrastructure.NAR_EMotor;
import net.thefletcher.revrobotics.enums.MotorType;

public class Climber extends SubsystemBase {
    
    public enum ClimberState {
        EXTENDED,
        RETRACTED;
    }

    private static Climber instance;

    private ClimberState climberState;

    private DoubleSolenoid m_climberSolenoid, m_climberBreakSolenoid;
    private NAR_CANSparkMax m_leftMotor, m_rightMotor;
    private DigitalInput m_leftLimitSwitch, m_rightLimitSwitch;

    public Climber() {
        climberState = ClimberState.RETRACTED;

        configMotors();
        configSensors();
        configPneumatics();

        resetLeftEncoder();
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

        m_leftMotor.setInverted(true);
        m_rightMotor.follow(m_leftMotor, true);
        
        m_leftMotor.setIdleMode(ClimberConstants.CLIMBER_NEUTRAL_MODE);
        m_rightMotor.setIdleMode(ClimberConstants.CLIMBER_NEUTRAL_MODE);

    }

    private void configSensors() {

        m_leftLimitSwitch = new DigitalInput(ClimberConstants.CLIMBER_SENSOR_LEFT_ID);
        m_rightLimitSwitch = new DigitalInput(ClimberConstants.CLIMBER_SENSOR_RIGHT_ID);
    }
    
    private void configPneumatics() {
        m_climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                ClimberConstants.CLIMBER_SOLENOID_FORWARD_CHANNEL_ID, 
                                                ClimberConstants.CLIMBER_SOLENOID_BACKWARD_CHANNEL_ID);
        m_climberBreakSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                ClimberConstants.CLIMBER_SOLENOID_BREAK_FORWARD_CHANNEL_ID, 
                                                ClimberConstants.CLIMBER_SOLENOID_BREAK_BACKWARD_CHANNEL_ID);

        retractPiston();
        disengageBreak();
    }

    @Override
    public void periodic() {

        if (getLeftSwitch() || getRightSwitch()) {
            //if state is retracted and encoder count is closer to top encoder count
            if (getState() == ClimberState.RETRACTED && getCurrentTicksLeft() > ClimberConstants.CLIMB_ENC_TO_TOP/2) {//Math.abs(getCurrentTicksLeft()) > Math.abs(ClimberConstants.CLIMB_ENC_TO_TOP)) {
                setState(ClimberState.EXTENDED);
            }
            //if state is extended and encoder count is closer to zero
            else if (getState() == ClimberState.EXTENDED && getCurrentTicksLeft() < ClimberConstants.CLIMB_ENC_TO_TOP/2) {//Math.abs(getCurrentTicksLeft()) < Math.abs(ClimberConstants.CLIMB_ENC_TO_TOP)){
                setState(ClimberState.RETRACTED);
                //resetLeftEncoder();
            }
        }
        

        // SmartDashboard.putString("Climber state", climberState.toString());

        SmartDashboard.putBoolean("Climber left limit switch", getLeftSwitch());
        SmartDashboard.putBoolean("Climber right limit switch", getRightSwitch());
        SmartDashboard.putNumber("Climber left encoder", getCurrentTicksLeft());
        SmartDashboard.putString("Climber pistons", m_climberSolenoid.get().toString());
        SmartDashboard.putString("Climber friction brake piston", m_climberBreakSolenoid.get().toString());

        // SmartDashboard.putNumber("Climber avgCurrent", getAvgCurrent());
    }

    public void bothExtend() {
        m_rightMotor.set(ClimberConstants.CLIMBER_POWER);
        m_leftMotor.set(ClimberConstants.CLIMBER_POWER);
    }

    public void bothRetract() {
        m_rightMotor.set(-ClimberConstants.CLIMBER_POWER);
        m_leftMotor.set(-ClimberConstants.CLIMBER_POWER);
    }

    public void bothManualExtend() {
        m_rightMotor.set(ClimberConstants.MANUAL_POWER);
        m_leftMotor.set(ClimberConstants.MANUAL_POWER);
    }

    public void bothManualRetract() {
        m_rightMotor.set(-ClimberConstants.MANUAL_POWER);
        m_leftMotor.set(-ClimberConstants.MANUAL_POWER);
    }

    public void bothStop() {
        m_rightMotor.set(0);
        m_leftMotor.set(0);
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

    public void setState(ClimberState state) {
        climberState = state;
    }

    public boolean getLeftSwitch() {
        return !m_leftLimitSwitch.get();
    }

    public boolean getRightSwitch() {
        return !m_rightLimitSwitch.get();
    }
    
    public double getDesiredTicks(double distance) {
        double desiredTicks = distance * (ConversionConstants.SPARK_ENCODER_RESOLUTION * ClimberConstants.CLIMBER_GEAR_RATIO) / (ClimberConstants.AXLE_DIAMETER * Math.PI);
        return desiredTicks;
    }

    public double getCurrentTicksLeft() {
        return m_leftMotor.getSelectedSensorPosition();
    }

    public ClimberState getState() {
        return climberState;
    }

    public void resetLeftEncoder() {
        m_leftMotor.setEncoderPosition(0);
    }

    public double getAvgCurrent() {
        return (m_leftMotor.getOutputCurrent() + m_rightMotor.getOutputCurrent()) / 2;
    }
}