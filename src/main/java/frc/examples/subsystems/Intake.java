package frc.examples.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.examples.hardware.motors.Simulable775ProTalonSRX;
import frc.examples.hardware.motors.Simulable775ProVictorSPX;
import frc.team3128.subsystems.Constants;

public class Intake extends SubsystemBase {
    private Simulable775ProTalonSRX armMotor;
    private Simulable775ProVictorSPX intakeRollerMotor;
    private Encoder armEncoder;
    private DigitalInput limitSwitchTop, limitSwitchBottom;

    public Intake() {
        super();
        // TODO Stuff
    }

    @Override
    protected void configActuators() {
        armMotor = new Simulable775ProTalonSRX(Constants.IntakeConstants.ARM_MOTOR_ID);
        intakeRollerMotor = new Simulable775ProVictorSPX(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    }

    @Override
    protected void configSensors() {
        // All of these sensors should also eventually be a Simulable version of this. 

        // I wasn't sure how this works for an encoder plugged directly into the TalonSRX.
        // Ideally the control is happening on board the motor controller because it can run at 1000Hz.
        // That would be using motion magic, I think.
        armEncoder = new Encoder(0, 1);
        double encoderTicksPerRevolution = 2048; // Example value, not unreasonable for an encoder
        armEncoder.setDistancePerPulse(360.0 / encoderTicksPerRevolution); // Converts pulses to angle when using getDistance()

        limitSwitchTop = new DigitalInput(Constants.IntakeConstants.TOP_LIMIT_SWITCH_ID);
        limitSwitchBottom = new DigitalInput(Constants.IntakeConstants.BOTTOM_LIMIT_SWITCH_ID);
    }

    @Override
    public void periodicUpdate() {
        if (getBottomLimitSwitch()) {
            stopArm();
        } else if (getBottomLimitSwitch()) {
            stopArm();
        }
    }

    public void moveArmUp() {
        armMotor.set(ControlMode.PercentOutput, -Constants.IntakeConstants.ARM_MOTOR_POWER);
    }

    public void moveArmDown() {
        armMotor.set(ControlMode.PercentOutput, -Constants.IntakeConstants.ARM_MOTOR_POWER);
    }

    public void stopArm() {
        armMotor.stop();
    }

    public boolean getBottomLimitSwitch() {
        return limitSwitchBottom.get();
    }

    public boolean getTopLimitSwitch() {
        return limitSwitchTop.get();
    }

    public void spinIntakeRollerIn() {
        intakeRollerMotor.set(Constants.IntakeConstants.INTAKE_MOTOR_POWER);
    }

    public void spinIntakeRollerOut() {
        intakeRollerMotor.set(-1 * Constants.IntakeConstants.INTAKE_MOTOR_POWER);
    }

    public void setIntakeRollerSpeed(float rpm) {
        // This should obviously be constants
        double intakeRollerGearing = 60/12 * 30 / 12; // Made up values
        
        double percentFreeSpeed = rpm / (intakeRollerMotor.getFreeSpeedRPM() / intakeRollerGearing);
        intakeRollerMotor.set(percentFreeSpeed * 12);
    }

    public double getArmAngle() {
        return armEncoder.getDistance();
    }
}
