package frc.examples.hardware.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Simulable775ProTalonSRX extends Simulable775ProBase<WPI_TalonSRX> {

    public Simulable775ProTalonSRX(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void constructReal() {
        this.motorController = new WPI_TalonSRX(deviceNumber);
    }

    @Override
    public void constructFake() {
        // TODO Construction of simulation items.
    }

    @Override
    public void set(double value) {
        set(TalonSRXControlMode.PercentOutput, value);
    }

    @Override
    public void set(ControlMode controlMode, double value) {
        set(TalonSRXControlMode.valueOf(controlMode.name()), value);
    }

    public void set(TalonSRXControlMode controlMode, double value) {
        set(controlMode, value, DemandType.Neutral, 0);
    }

    public void set(TalonSRXControlMode controlMode, double value, DemandType demandType, double demandValue) {
        getMotorController().set(controlMode, value, demandType, demandValue);
    }
}