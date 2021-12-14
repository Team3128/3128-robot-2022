package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;


public class Constants {

    public static class ShooterConstants {

        public static final double SHOOTER_PID_kP = 1.24e-7; //1.24e-7
        public static final double SHOOTER_PID_kI = 0;
        public static final double SHOOTER_PID_kD = 0;
        public static final double SHOOTER_PID_kF = 0;

        public static final int LEFT_SHOOTER_ID = 8; //Left Shooter Motor
        public static final int RIGHT_SHOOTER_ID = 13; //Right Shooter Motor

        public static final int PLATEAU_COUNT = 25; //Number of checks at correct RPM to shoot
        public static final double THRESHOLD_PERCENT = 0.1; //0.05; //Maximum Percent Error in RPM to still shoot
        public static final double RPM_THRESHOLD_PERCENT = 0.05;
        public static final double RPM_THRESHOLD_PERCENT_MAX = 0.1;
        public static final double TIME_TO_MAX_THRESHOLD = 5;

        public static final double SHOOTER_KS = 0.711; //Static gain in PID Feed Forward
        public static final double SHOOTER_KV = 0.00176; //0.00163; //Velocity gain in PID Feed Forward
        public static final double SHOOTER_KA = 0.0349; //Acceleration gain PID Feed Forward

        // There are other better ways of doing this, need to use DCU 
        public static final LinearSystem<N1, N1, N1> SHOOTER_CHAR = 
        LinearSystemId.identifyVelocitySystem(
            SHOOTER_KV, 
            SHOOTER_KA
        );
        public static final DCMotor SHOOTER_GEARBOX = DCMotor.getFalcon500(2);
        public static final double SHOOTER_GEARING = 1.5;
        public static final double SHOOTER_RADIUS_METERS = 0.0508; //not correct
  
       
    }

    public static class SidekickConstants {

        public static final int SIDEKICK_ID = 42;

        public static final float SIDEKICK_PID_kP = 0;
        public static final float SIDEKICK_PID_kI = 0;
        public static final float SIDEKICK_PID_kD = 0;

        public static final int SIDEKICK_RPM_THRESHOLD_PERCENT = 0;
        public static final double SIDEKICK_PLATEAU_THRESHOLD = 5;

        public static final int CAN_TIMEOUT = 0;
        public static final double SIDEKICK_kS = 0;
        public static final double SIDEKICK_kV = 0;

        public static final double SIDEKICK_UNITS_PER_ROTATION = 4096;

    }

    public static class ConversionConstants {
        public static final double FALCON_ENCODER_RESOLUTION = 2048;
        public static final double ENCODER_TO_RPM = 10*60/FALCON_ENCODER_RESOLUTION; // (sensor units per 100 ms to rpm)
        public static final double SIDEKICK_ENCODER_TO_RPM = 10*60/SidekickConstants.SIDEKICK_UNITS_PER_ROTATION;

    }

    public static class DriveConstants {


        // Sim constants, TODO: move to new class
        public static final DCMotor GEARBOX = DCMotor.getFalcon500(4); 
        public static final LinearSystem<N2, N2, N2> DRIVE_CHAR = 
        LinearSystemId.identifyDrivetrainSystem(
            5, //0.5, // kvVoltSecondsPerMeter
            0.5,//0.05, // kaVoltSecondsSquaredPerMeter
            5,//1.5, // kvVoltSecondsPerRadian
            0.5//0.3 // kaVoltSecondsSquaredPerRadian
        );
        public static final double DRIVE_GEARING = 8;
        public static final double WHEEL_RADIUS_METERS = 0.0508;

        public static final int DRIVE_MOTOR_LEFT_LEADER_ID = 0;
        public static final int DRIVE_MOTOR_LEFT_FOLLOWER_ID = 1;
        public static final int DRIVE_MOTOR_RIGHT_LEADER_ID = 2;
        public static final int DRIVE_MOTOR_RIGHT_FOLLOWER_ID = 3;

        public static final boolean GYRO_REVERSED = false;
        public static final double TRACK_WIDTH_METERS = 0.66;

        
        public static final double MAX_DRIVE_VELOCITY = 1.75; // m/s
        public static final double MAX_DRIVE_ACCELERATION = 1.5; // m/s^2

        public static final int WHEEL_RADIUS = 2; // inches
        public static final double TRACK_WIDTH = 0.66; // meters
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

        public static final double ENCODER_RESOLUTION_PER_ROTATION = 2048;
        public static final double ENCODER_DISTANCE_PER_MARK = WHEEL_RADIUS * 2 * Math.PI / ENCODER_RESOLUTION_PER_ROTATION;

        public static final double ARCADE_DRIVE_TURN_MULT = -0.6; //-0.7

        public static final double kS = 0.8; // 0.73; // volts
        public static final double kV = 0.5; // 0.571; // volt*seconds/meter
        public static final double kA = 0.05; // 0.013; // volt*seconds^2/meter

        public static final double MAX_DRIVE_VOLTAGE = 11; // volts (hopefully you could figure this out)
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;
        public static final double RAMSETE_KP = 0.5; // I think Tyler just took this number and ran with it

    }

    public static class IntakeConstants {
        public static final int ARM_MOTOR_ID = 7;
        public static final int BRUSH_MOTOR_1_ID = 5;
        public static final int BRUSH_MOTOR_2_ID = 33;
        public static final int INTAKE_MOTOR_ID = 4;

        //Chech these, for some reason Nathan had them as 0 and 1
        public static final int TOP_LIMIT_SWITCH_ID = 0;
        public static final int BOTTOM_LIMIT_SWITCH_ID = 1;

        public static final double INTAKE_MOTOR_POWER = 0.6;
        public static final double BRUSH_MOTOR_POWER = 0.3;
        public static final double ARM_MOTOR_POWER = 0.42;

        public static final double ARM_MOTOR_POWER_AUTO = 0.07;
        public static final NeutralMode ARM_NEUTRAL_MODE = NeutralMode.Brake;
    }

    public static class HopperConstants {

        public static final int HOPPER_MOTOR_1_ID = 6;
        public static final int HOPPER_MOTOR_2_ID = 19;

        public static final int BOTTOM_SENSOR_ID = 9;
        public static final int TOP_SENSOR_ID = 8;

        public static final double HOPPER_MOTOR_1_POWER = -0.5;
        public static final double HOPPER_MOTOR_2_POWER = -0.3;
        

    }
  
    public static class ClimberConstants {

        public static final int CLIMBER_MOTOR_1_ID = 10;
        public static final int CLIMBER_MOTOR_2_ID = 11;

        public static final double CLIMBER_UP_POWER = 1;
        public static final double CLIMBER_DOWN_POWER = -1;
    
        public static final NeutralMode CLIMBER_NEUTRAL_MODE = NeutralMode.Brake;
    }

    public static class VisionContants {

        public static final String TOP_HOSTNAME = "10.31.28.25";

        public static final int SAMPLE_RATE = 3;

        public static final double TOP_CAMERA_ANGLE = -26.0; //degrees
        public static final double TOP_CAMERA_HEIGHT = 0.0; // Daniel - We had this at 0.0 previously, if we want to do more advanced math using vision this value should be measured - also determine units
        public static final double TOP_FRONT_DIST = 0.0; // Daniel - We had this at 0.0 previously, if we want to do more advanced math using vision this value should be measured.
        public static final double TARGET_WIDTH = 30.0; //inches

        public static final double VISION_PID_kP = 0.02;
        public static final double VISION_PID_kI = 0.02;
        public static final double VISION_PID_kD = 0.00006;

        public static final double TX_OFFSET = 0.0; // to offset alignment in either direction

        public static final double TX_THRESHOLD = 2; //degrees
        public static final double TX_THRESHOLD_MAX = 5; //degrees
        public static final double TIME_TO_MAX_THRESHOLD = 5; //seconds
        public static final double TX_THRESHOLD_INCREMENT = (TX_THRESHOLD_MAX - TX_THRESHOLD) / TIME_TO_MAX_THRESHOLD; //degrees per second

        public static final int ALIGN_PLATEAU_COUNT = 10; //Number of checks at correct RPM to shoot
        public static final double ALIGN_FF = 0.1;
        
    }
}
