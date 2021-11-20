package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3128.hardware.NAR_TalonSRX;

public class Hopper implements Subsystem{
    public static final Hopper instance = new Hopper();
    private NAR_TalonSRX HOPPER_MOTOR_1;
    // HOPPER_MOTOR_2 is a cansparkmax - kayoon is getting the code

    private DigitalInput BOTTOM_SENSOR, TOP_SENSOR;
    private boolean isShooting;

    public Hopper() {
        configMotors();
        configSensors();
    }

    private void configMotors() {
        HOPPER_MOTOR_1 = new NAR_TalonSRX(Constants.HopperConstants.HOPPER_MOTOR_1_ID);
        // config hopper motor 2
    }

    private void configSensors() {
        BOTTOM_SENSOR = new DigitalInput(Constants.HopperConstants.BOTTOM_SENSOR_ID);
        TOP_SENSOR = new DigitalInput(Constants.HopperConstants.TOP_SENSOR_ID);
    }

    @Override
    public void periodic() {
        if (isShooting) {
            runHopper(1);
        }
        else {
            if (getBottom() && !getTop()) {
                runHopper(1);
            }
            else {
                stopHopper();
            }
        }
    }

    public void setisShooping(boolean isShooting) {
        this.isShooting = isShooting;
    }

    public boolean getisShooting() {
        return isShooting;
    }

    private boolean getBottom() {
        return !BOTTOM_SENSOR.get();
    }

    private boolean getTop() {
        return !TOP_SENSOR.get();
    }

    public void runHopper(double multiplier) {
        HOPPER_MOTOR_1.set(ControlMode.PercentOutput, Constants.HopperConstants.HOPPER_MOTOR_1_POWER*multiplier);
        //HOPPER_MOTOR_2.set(Constants.HopperConstants.HOPPER_MOTOR_2_POWER);
    }

    public void stopHopper() {
        HOPPER_MOTOR_1.set(ControlMode.PercentOutput, 0);
        //HOPPER_MOTOR_2.set(0);
    }

}
