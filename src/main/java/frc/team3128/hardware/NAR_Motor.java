package frc.team3128.hardware;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.SpeedController;
import frc.team3128.common.Simulable;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.SpeedController;

public abstract class NAR_Motor<T extends SpeedController> extends Simulable {

    // Condensed the contant-storing functionality of this class to an enum
    public static enum MotorConstants{

        Vex775Pro(18730, 0.7, 134, 0.71);

        private double freeSpeedRPM;
        private double freeCurrentAmps;
        private double stallCurrentAmps;
        private double stallTorqueNM;

        MotorConstants(double freeSpeedRPM, double freeCurrentAmps, double stallCurrentAmps, double stallTorqueNM){
            this.freeSpeedRPM = freeSpeedRPM;
            this.freeCurrentAmps = freeCurrentAmps;
            this.stallCurrentAmps = stallCurrentAmps;
            this.stallTorqueNM = stallTorqueNM;
        }

        public double getFreeSpeedRPM() {
            return freeSpeedRPM;
        }
    
        public double getFreeCurrentAmps() {
            return freeCurrentAmps;
        }
    
        public double getStallCurrentAmps () {
            return stallCurrentAmps;
        }
    
        public double getStallTorqueNM() {
            return stallTorqueNM;
        }
    }

    public static enum MotorControllerType {
        TALON_FX, TALON_SRX, VICTOR_SPX;
    }

    protected int deviceNumber;
    protected T motorController;
    protected SimDevice encoderSim;
    protected SimDouble simPos;
    protected SimDouble simVel;

    

    protected NAR_Motor(int deviceNumber) {
        this.deviceNumber = deviceNumber;
        construct();
    }

    public static NAR_Motor create(int deviceNumber, MotorControllerType motorType) {
        NAR_Motor result;
        switch(motorType) {
            case TALON_FX:
                result = new NAR_TalonFX(deviceNumber);
                break;
            case TALON_SRX:
                result =  new NAR_TalonSRX(deviceNumber);
                break;
            case VICTOR_SPX:
                result = new NAR_VictorSPX(deviceNumber);
                break;
            default:
                return null;
        }

        result.construct();
        return result;
    }

    // Need clarification on how to use this
    @Override
    public void updateSimulation(double timeStep) {
        
    }

    // Probably how this should be implemented
    @Override
    public void constructFake(){
        encoderSim = SimDevice.create("Encoder ["+deviceNumber+"]", deviceNumber);
        simPos = encoderSim.createDouble("Pos", Direction.kBidir, 0);
        simVel = encoderSim.createDouble("Vel", Direction.kBidir, 0);
    }

    public abstract void set(double value);
    /**
     * Set control with lazy logic
     * @param controlMode
     * @param value
     */
    public abstract void set(ControlMode controlMode, double value);

    public abstract double getSetpoint();

    public void stop() {
        motorController.stopMotor();
    }

    public T getMotorController() {
        return (T) motorController;
    } 

    public SimDevice getFakeMotor(){
        return encoderSim;
    }
}