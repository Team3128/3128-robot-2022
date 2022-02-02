package frc.team3128;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

// CURRENTLY CONFIGURED FOR 4 FALCON DRIVE (Speedy G)

public class Constants {

    public static class ConversionConstants {

        public static final double FALCON_ENCODER_RESOLUTION = 2048;
        public static final double SPARK_ENCODER_RESOLUTION = 42;
        public static final double SPARK_VELOCITY_FACTOR = SPARK_ENCODER_RESOLUTION / 60; // rmp to nu/s
        public static final double FALCON_NUp100MS_TO_RPM = 10 * 60 / FALCON_ENCODER_RESOLUTION; // sensor units per 100 ms to rpm
    }

    public static class DriveConstants {

        public static final int DRIVE_MOTOR_LEFT_LEADER_ID = 0;
        public static final int DRIVE_MOTOR_LEFT_FOLLOWER_ID = 1;
        public static final int DRIVE_MOTOR_RIGHT_LEADER_ID = 2;
        public static final int DRIVE_MOTOR_RIGHT_FOLLOWER_ID = 3;

        public static final int KIT_MOTOR_LEFT_LEADER_ID = 1;
        public static final int KIT_MOTOR_LEFT_FOLLOWER_ID = 2;
        public static final int KIT_MOTOR_RIGHT_LEADER_ID = 3;
        public static final int KIT_MOTOR_RIGHT_FOLLOWER_ID = 4;

        public static final double DRIVE_GEARING = 9.6;
        public static final double WHEEL_RADIUS_METERS = 0.0762; // 3 inches
        public static final double TRACK_WIDTH_METERS = 0.59312;

        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
        public static final double ENCODER_DISTANCE_PER_MARK = WHEEL_RADIUS_METERS * 2 * Math.PI / ConversionConstants.FALCON_ENCODER_RESOLUTION;
        public static final double DRIVE_DIST_PER_TICK = ENCODER_DISTANCE_PER_MARK / DRIVE_GEARING; // meters per encoder tick

        public static final double kS = 0.63899;
        public static final double kV = 2.1976;
        public static final double kA = 0.12023;
        public static final double kVAngular = 1.5;       // Nathan's magic numbers of doom
        public static final double kAAngular = 0.3;     // Nathan's magic numbers of doom

        public static final double MAX_DRIVE_VELOCITY = 4; // m/s - Real value ~5
        public static final double MAX_DRIVE_ACCELERATION = 2; // m/s^2 - I don't know what this number is
        public static final double MAX_DRIVE_VOLTAGE = 7; // volts (hopefully you could figure this out)

        //Ramsete constants
        public static final double RAMSETE_B = 2; //default value - don't change unless absolutely necessary
        public static final double RAMSETE_ZETA = 0.7; //default value - don't change unless absolutely necessary
        public static final double RAMSETE_KP = 2.1963;

        public static final Boolean GYRO_REVERSED = false;

        public static final DCMotor GEARBOX = DCMotor.getFalcon500(4); 
        public static final LinearSystem<N2, N2, N2> DRIVE_CHAR = 
        LinearSystemId.identifyDrivetrainSystem(
            kV,                 // kvVoltSecondsPerMeter
            kA,                 // kaVoltSecondsSquaredPerMeter
            kVAngular,          // kvVoltSecondsPerRadian
            kAAngular           // kaVoltSecondsSquaredPerRadian
        );
    }

    public static class ClimberConstants {

        public static final int CLIMBER_SENSOR_ID = 0;
        public static final int CLIMBER_MOTOR_1_ID = 6;
        public static final int CLIMBER_MOTOR_2_ID = 7;

        public static final int CLIMBER_SOLENOID_FORWARD_CHANNEL_ID = 1;
        public static final int CLIMBER_SOLENOID_BACKWARD_CHANNEL_ID = 2;
        public static final int CLIMBER_SOLENOID_BREAK_FORWARD_CHANNEL_ID = 7;
        public static final int CLIMBER_SOLENOID_BREAK_BACKWARD_CHANNEL_ID = 8;

        public static final double CLIMBER_GEAR_RATIO = 18.9;
        public static final double AXLE_DIAMETER = 0.7;
        public static final double CLIMBER_ERROR_RATE = .5; //in inches

        public static final double CLIMBER_HEIGHT = 20; // inches

        public static final double VERTICAL_DISTANCE = 24;  // TODO: change to actual distance
        public static final double SMALL_VERTICAL_DISTANCE = 6; // TODO: change to actual distance
        public static final double ANGLED_DISTANCE = 12; // TODO: change to actual distance
    

        public static final NeutralMode CLIMBER_NEUTRAL_MODE = null;
        public static final double CLIMBER_POWER = 0.5;
    }

    public static class ShooterConstants {

        public static final int LEFT_SHOOTER_ID = 8; 
        public static final int RIGHT_SHOOTER_ID = 9; 


        public static final double SHOOTER_PID_kP = 1.24e-3;
        public static final double SHOOTER_PID_kI = 0;
        public static final double SHOOTER_PID_kD = 0;

        public static final double SHOOTER_KS = 0.711; //Static gain in PID Feed Forward
        public static final double SHOOTER_KV = 0.00163; //Velocity gain in PID Feed Forward
        public static final double SHOOTER_KA = 0.0349; //Acceleration gain PID Feed Forward

        public static final int PLATEAU_COUNT = 25;

        public static final double RPM_THRESHOLD_PERCENT = 0.05;
        public static final double RPM_THRESHOLD_PERCENT_MAX = 0.06;
        public static final double TIME_TO_MAX_THRESHOLD = 8;

        public static final LinearSystem<N1, N1, N1> SHOOTER_CHAR = 
        LinearSystemId.identifyVelocitySystem(
            SHOOTER_KV, 
            SHOOTER_KA
        );
        public static final double SHOOTER_RADIUS_METERS = 0.0508;
        public static final DCMotor SHOOTER_GEARBOX = DCMotor.getCIM(2);
        public static final double SHOOTER_GEARING = 1.5;
    }
  
    public static class HopperConstants {

        public static final int HOPPER_MOTOR_ID = 5;
        // public static final int BOTTOM_SENSOR_ID = 12;
        // public static final int TOP_SENSOR_ID = 13;

        public static final int HOPPER_SOLENOID_FORWARD_CHANNEL_ID = 3;
        public static final int HOPPER_SOLENOID_BACKWARD_CHANNEL_ID = 4;

        public static final int HOPPER_DIO_PIN1 = 0;
        public static final int HOPPER_DIO_PIN2 = 1;
        public static final int HOPPER_MAX_REVERSE_DISTANCE = 0; //set distance

        public static final double HOPPER_MOTOR_POWER = 0.5;
        public static final double REVERSE_HOPPER_MOTOR_POWER = 0.5;
    }

    public static class IntakeConstants {

        public static final int INTAKE_MOTOR_ID = 5; 
        public static final int INTAKE_SOLENOID_FORWARD_CHANNEL_ID = 3;
        public static final int INTAKE_SOLENOID_BACKWARD_CHANNEL_ID = 4;

        public static final double INTAKE_MOTOR_POWER = -1; //negative is forward 

    }

    public static class VisionContants {

        public static final String TOP_HOSTNAME = "limelight-sog";

        public static final int SAMPLE_RATE = 3;

        public static final double TOP_CAMERA_ANGLE = -26.0; //degrees
        public static final double TOP_CAMERA_HEIGHT = 0.0; // Daniel - We had this at 0.0 previously, if we want to do more advanced math using vision this value should be measured - also determine units
        public static final double TOP_FRONT_DIST = 0.0; // Daniel - We had this at 0.0 previously, if we want to do more advanced math using vision this value should be measured.
        public static final double TARGET_WIDTH = 30.0; //inches

        public static final double VISION_PID_kP = 0.01;
        public static final double VISION_PID_kI = 0.02;
        public static final double VISION_PID_kD = 0.00006;

        public static final double TX_OFFSET = 0.0; // to offset alignment in either direction

        public static final double TX_THRESHOLD = 1; //degrees
        public static final double TX_THRESHOLD_MAX = 2; //degrees
        public static final double TIME_TO_MAX_THRESHOLD = 5; //seconds
        public static final double TX_THRESHOLD_INCREMENT = (TX_THRESHOLD_MAX - TX_THRESHOLD) / TIME_TO_MAX_THRESHOLD; //degrees per second

        public static final int ALIGN_PLATEAU_COUNT = 10; //Number of checks at correct RPM to shoot
    }
}
