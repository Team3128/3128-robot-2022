package frc.team3128.common;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.ctre.phoenix.motorcontrol.ControlMode;

public abstract class NAR_Motor<T extends MotorController> extends Simulable {

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

    protected int deviceNumber;
    protected T motorController;
    protected SimDevice encoderSim;
    protected SimDouble simPos;
    protected SimDouble simVel;

    private NAR_Motor(int deviceNumber) {
        this.deviceNumber = deviceNumber;
        construct();
    }

    @Override
    public void updateSimulation(double timeStep) {
        
    }

    @Override
    public void constructFake(){
        encoderSim = SimDevice.create("Encoder ["+deviceNumber+"]", deviceNumber);
        simPos = encoderSim.createDouble("Pos", Direction.kBidir, 0);
        simVel = encoderSim.createDouble("Vel", Direction.kBidir, 0);
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