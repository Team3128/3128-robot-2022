
package frc.team3128.common.hardware.motor;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class NAR_VictorSPX extends WPI_VictorSPX{

    public NAR_VictorSPX(int deviceNumber, boolean isLazy) {
        super(deviceNumber);
        enableVoltageCompensation(true);
        configVoltageCompSaturation(12, 10);
        lazy = isLazy;
    }

        private boolean lazy;
        private double prevValue = 0;
        private ControlMode prevControlMode = ControlMode.Disabled;
    
        /**
         * 
         * @param deviceNumber device id
         * @param isLazy whether or not the motor is lazy
         */
    
    
        /**
         * 
         * @param deviceNumber device id
         */
        public NAR_VictorSPX(int deviceNumber){
            this(deviceNumber, true);
        }
    
        public void setLazy(boolean isLazy){
            lazy = isLazy;
        }
    
        public boolean isLazy(){
            return lazy;
        }
    
        @Override
        public void set(ControlMode controlMode, double outputValue) {
            if(lazy){
                if (outputValue != prevValue || controlMode != prevControlMode) {
                    super.set(controlMode, outputValue);
                    prevValue = outputValue;
                }
            }else{
                super.set(controlMode, outputValue);
            }
        }
    
        public double getSetpoint() {
            return prevValue;
        }
    }
    

