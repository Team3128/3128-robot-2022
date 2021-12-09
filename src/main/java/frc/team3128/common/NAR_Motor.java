package frc.team3128.common;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.SpeedController;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.fasterxml.jackson.databind.deser.AbstractDeserializer;

import edu.wpi.first.wpilibj.SpeedController;

public abstract class NAR_Motor<T extends SpeedController> extends Simulable {

    // Condensed the contant-storing functionality of this class to an enum
    public static enum MotorType{

        Vex775Pro(18730, 0.7, 134, 0.71);

        private double freeSpeedRPM;
        private double freeCurrentAmps;
        private double stallCurrentAmps;
        private double stallTorqueNM;

        MotorType(double freeSpeedRPM, double freeCurrentAmps, double stallCurrentAmps, double stallTorqueNM){
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

    // Have to be defined higher up
    protected double encoderRes;
    protected double moi;

    protected MotorType type;
    protected int deviceNumber;
    protected T motorController;
    protected SimDevice encoderSim;
    protected SimDouble simPos;
    protected SimDouble simVel;
    protected SimDouble simLoad;

    private NAR_Motor(int deviceNumber) {
        this.deviceNumber = deviceNumber;
        construct();
    }

    /**
     * Simulates physics for an individual motor assuming perfect conditions
     */
    @Override
    public void updateSimulation(double timeStep) {
        double position = simPos.get();
        double velocity = simVel.get();

        velocity+=getSimAcc()*timeStep;

        simVel.set(velocity);
        simPos.set(position+velocity*timeStep);
    }

    @Override
    public void constructFake(){
        encoderSim = SimDevice.create(motorController.getClass().getSimpleName()+"["+deviceNumber+"] simEncoder", deviceNumber);
        simPos = encoderSim.createDouble("Pos", Direction.kBidir, 0);
        simVel = encoderSim.createDouble("Vel", Direction.kBidir, 0);
        simLoad = encoderSim.createDouble("Load", Direction.kBidir, 0);
    }

    /**
     * @return Position in native units
     */
    public double getSimPos(){
        return simPos.get();
    }

    /**
     * @return Velocity in native units / second
     */
    public double getSimVel(){

        // freeSpeed in native units / second
        double freeSpeed = (type.getFreeSpeedRPM() * encoderRes / 60);

        if(simVel.get() < freeSpeed)
            return simVel.get();
        else
            return freeSpeed;
    }

    /**
     * @return Acceleration in native units / second squared
     */
    public double getSimAcc(){
        return getSimTorque() / moi;
    }

    /**
     * @return Net torque in N*m
     */
    public double getSimTorque(){

        double appTorq = type.getStallTorqueNM()*motorController.get();

        appTorq *= 1 - Math.abs(getSimVel()) / (type.getFreeSpeedRPM() * encoderRes / 60);

        return appTorq - simLoad.get();
    }

    /**
     * IMPLEMENT WITH CAUTION, this may require knowledge of more complete motor state
     * 
     * @param load simulated load in N*m
     */
    public void setSimLoad(double load){
        simLoad.set(load);
    }

    public abstract void set(double value);
    public abstract void set(ControlMode controlMode, double value);

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