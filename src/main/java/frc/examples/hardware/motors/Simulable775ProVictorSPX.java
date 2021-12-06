package frc.examples.hardware.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Simulable775ProVictorSPX extends Simulable775ProBase<WPI_VictorSPX> {

    public Simulable775ProVictorSPX(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void constructReal() {
        this.motorController = new WPI_VictorSPX(deviceNumber);
    }

    @Override
    public void constructFake() {
        // TODO Construction of simulation items.
    }

    @Override
    public void set(double value) {
        set(ControlMode.PercentOutput, value);
    }

    @Override
    public void set(ControlMode controlMode, double value) {
        set(VictorSPXControlMode.valueOf(controlMode.name()), value);
    }

    public void set(VictorSPXControlMode controlMode, double value) {
        set(controlMode, value, DemandType.Neutral, 0);
    }

    public void set(VictorSPXControlMode controlMode, double value, DemandType demandType, double demandValue) {
        getMotorController().set(controlMode, value, demandType, demandValue);
    }
}