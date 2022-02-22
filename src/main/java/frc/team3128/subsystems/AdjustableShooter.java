package frc.team3128.subsystems;

import frc.team3128.Constants.AdjustableShooterConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import net.thefletcher.revrobotics.enums.MotorType;

//Shooter Hood
//CmdShoot
//Constants (possibly create a new subsection)

public class AdjustableShooter extends SubsystemBase{

    private static AdjustableShooter instance;
    private NAR_CANSparkMax m_hoodShooter;

    // public enum ShooterAngle {
    //     UPPER,
    //     LOWER;

    //     // public double shooterAngle;

    //     // private ShooterAngle(double angle) {
    //     //     this.shooterAngle = angle;
    //     // }
    // }

    //configuration
    private DigitalInput m_hoodState;
    public AdjustableShooter() {
        configMotors();
        configSensors();
    }

    //instance
    public static synchronized AdjustableShooter getInstance() {
        if(instance == null) {
            instance = new AdjustableShooter();
        }
        return instance;
    }

    //motor
    private void configMotors() {
        m_hoodShooter = new NAR_CANSparkMax(AdjustableShooterConstants.HOOD_SHOOTER_ID, MotorType.kBrushless); //double check
    }

    //sensors
    private void configSensors() {
        m_hoodState = new DigitalInput(AdjustableShooterConstants.HOOD_SENSOR_ID);
    }

    //methods for movement
    public void hoodUp() {
        m_hoodShooter.set(AdjustableShooterConstants.HOOD_SHOOTER_SPEED);
    }
    public void hoodStop(){
        m_hoodShooter.set(0);
    }
    public void hoodDown(){
        m_hoodShooter.set(-(AdjustableShooterConstants.HOOD_SHOOTER_SPEED));
    }

    //return sensors
    public boolean getHoodSwitch() {
        return m_hoodState.get();
    }

    // desired distance
    //  public double getDesiredTicks(double distance) {
    //      double desiredTicks = distance; //math bad
    //      return desiredTicks;
    // }

    //current distance
    public double getCurrentTicksLeft() {
        return (m_hoodShooter.getSelectedSensorPosition());
    }

    //stopping if checking is above/below? 
    public void periodic() {
        //Narwhal dashboard
        SmartDashboard.putString("HoodState", m_hoodState.toString());
    }
}