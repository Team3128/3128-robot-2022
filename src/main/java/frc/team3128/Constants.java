package frc.team3128;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import net.thefletcher.revrobotics.enums.IdleMode;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

// CURRENTLY CONFIGURED FOR 4 FALCON DRIVE (Speedy G)

public class Constants {

    public static class ConversionConstants {

        public static final double FALCON_ENCODER_RESOLUTION = 2048;
        public static final double SPARK_ENCODER_RESOLUTION = 42;
        public static final double SPARK_VELOCITY_FACTOR = SPARK_ENCODER_RESOLUTION / 60; // RPM to nu/s
        public static final double FALCON_NUp100MS_TO_RPM = 10 * 60 / FALCON_ENCODER_RESOLUTION; // sensor units per 100 ms to rpm
        public static final double FALCON_NUpS_TO_RPM = 60 / FALCON_ENCODER_RESOLUTION; // sensor units per second to rpm

        public static final double INCHES_TO_METERS = 0.0254;
    }

    public static class DriveConstants {

        public static final int DRIVE_MOTOR_LEFT_LEADER_ID = 0;
        public static final int DRIVE_MOTOR_LEFT_FOLLOWER_ID = 1;
        public static final int DRIVE_MOTOR_RIGHT_LEADER_ID = 2;
        public static final int DRIVE_MOTOR_RIGHT_FOLLOWER_ID = 3;

        // public static final int KIT_MOTOR_LEFT_LEADER_ID = 1;
        // public static final int KIT_MOTOR_LEFT_FOLLOWER_ID = 2;
        // public static final int KIT_MOTOR_RIGHT_LEADER_ID = 3;
        // public static final int KIT_MOTOR_RIGHT_FOLLOWER_ID = 4;

        public static final double ARCADE_DRIVE_TURN_MULT = 0.7;

        // Drive characterization - taken from Speedy Gonzales
        public static final double DRIVE_GEARING = 9.6;
        public static final double WHEEL_RADIUS_METERS = 0.0762; // 3 inches
        public static final double TRACK_WIDTH_METERS = 0.59312;

        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
        public static final double ENCODER_DISTANCE_PER_MARK = WHEEL_RADIUS_METERS * 2 * Math.PI / ConversionConstants.FALCON_ENCODER_RESOLUTION;
        public static final double DRIVE_NU_TO_METER = ENCODER_DISTANCE_PER_MARK / DRIVE_GEARING; // meters driven per encoder tick
        public static final double DRIVE_NUp100MS_TO_MPS = DRIVE_NU_TO_METER * 10; // sensor units per 100 ms to m/s of drivetrain
        public static final double MAX_DRIVE_VEL_NUp100MS = 6380 * ConversionConstants.FALCON_ENCODER_RESOLUTION / 60 / 10; // max angular velocity of drivetrain (encoder, not wheel) in sensor units per 100 ms - 6380 RPM * RESOLUTION nu/rot * 1 min/60s * 1s/(10*100ms)

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

        public static final int CLIMBER_SENSOR_LEFT_ID = 0;
        public static final int CLIMBER_SENSOR_RIGHT_ID = 1;
        public static final int CLIMBER_MOTOR_LEFT_ID = 6;
        public static final int CLIMBER_MOTOR_RIGHT_ID = 7;

        public static final int CLIMBER_SOLENOID_FORWARD_CHANNEL_ID = 1;
        public static final int CLIMBER_SOLENOID_BACKWARD_CHANNEL_ID = 14;
        public static final int CLIMBER_SOLENOID_BREAK_FORWARD_CHANNEL_ID = 2;
        public static final int CLIMBER_SOLENOID_BREAK_BACKWARD_CHANNEL_ID = 13;

        public static final double CLIMBER_GEAR_RATIO = 18.9;
        public static final double AXLE_DIAMETER = 0.7;
        public static final double CLIMBER_ERROR_RATE = .5; //in inches

        public static final double CLIMBER_HEIGHT = 20; // inches

        public static final double VERTICAL_DISTANCE = 24;  // TODO: change to actual distance
        public static final double SMALL_VERTICAL_DISTANCE = 6; // TODO: change to actual distance
        public static final double ANGLED_DISTANCE = 12; // TODO: change to actual distance
    
        public static final IdleMode CLIMBER_NEUTRAL_MODE = IdleMode.kBrake;
        public static final double CLIMBER_POWER = 0.6;
    }

    public static class ShooterConstants {

        public static final int LEFT_SHOOTER_ID = 4; 
        public static final int RIGHT_SHOOTER_ID = 5; 

        public static final double SHOOTER_PID_kP = 1.24e-5;//0.21576; // 1.24e-3;
        public static final double SHOOTER_PID_kI = 0;
        public static final double SHOOTER_PID_kD = 0;

        public static final double SHOOTER_KS = 0.3; // 0.711; //Static gain in PID Feed Forward
        public static final double SHOOTER_KV = 0.10714 / 60; // 0.00163; //Velocity gain in PID Feed Forward
        public static final double SHOOTER_KA = 0.0053359; // 0.0349; //Acceleration gain PID Feed Forward

        public static final int PLATEAU_COUNT = 1;

        public static final double RPM_THRESHOLD_PERCENT = 0.05;
        public static final double RPM_THRESHOLD_PERCENT_MAX = 0.11;
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

        public static final int HOPPER_MOTOR_ID = 6;
        // public static final int BOTTOM_SENSOR_ID = 12;
        // public static final int TOP_SENSOR_ID = 13;

        public static final int HOPPER_SOLENOID_FORWARD_CHANNEL_ID = 5;
        public static final int HOPPER_SOLENOID_BACKWARD_CHANNEL_ID = 6;

        public static final int HOPPER_DIO_PIN1 = 8;
        public static final int HOPPER_DIO_PIN2 = 9;
        public static final double HOPPER_MAX_REVERSE_DISTANCE = -1; //set distance

        public static final double HOPPER_MOTOR_POWER = 0.5;
        public static final double REVERSE_HOPPER_MOTOR_POWER = -1; //change this
    }

    public static class IntakeConstants {

        public static final int INTAKE_MOTOR_ID = 7; 
        public static final int INTAKE_SOLENOID_FORWARD_CHANNEL_ID = 12;
        public static final int INTAKE_SOLENOID_BACKWARD_CHANNEL_ID = 3;

        public static final double INTAKE_MOTOR_POWER = -1; //negative is forward 

    }

    public static class VisionConstants {

        public static final String TOP_HOSTNAME = "limelight-sog";

        public static final int SAMPLE_RATE = 3;

        public static final double TOP_CAMERA_ANGLE = (90 - 34.8519) * Math.PI / 180; // radians
        public static final double TOP_CAMERA_HEIGHT = 26; // in 
        public static final double TOP_FRONT_DIST = 0;
        public static final double TARGET_HEIGHT = 104;

        public static final double VISION_PID_kP = 0.01;
        public static final double VISION_PID_kI = 0.02;
        public static final double VISION_PID_kD = 0.00006;
        public static final double VISION_PID_kF = 0.0;

        public static final double TX_OFFSET = 0.0; // to offset alignment in either direction

        public static final double TX_THRESHOLD = 1; //degrees
        public static final double TX_THRESHOLD_MAX = 2; //degrees
        public static final double TIME_TO_MAX_THRESHOLD = 5; //seconds
        public static final double TX_THRESHOLD_INCREMENT = (TX_THRESHOLD_MAX - TX_THRESHOLD) / TIME_TO_MAX_THRESHOLD; //degrees per second

        public static final int ALIGN_PLATEAU_COUNT = 10; //Number of checks at correct RPM to shoot

        public static final double BALL_TARGET_HEIGHT = 9.5 * ConversionConstants.INCHES_TO_METERS;
        public static final double BALL_LL_HEIGHT = 21 * ConversionConstants.INCHES_TO_METERS;
        public static final double BALL_LL_ANGLE = 65.15 * Math.PI / 180; // 1.0; // Math.acos(21.0 / 39.0); // 1.002186; // radians
        public static final double BALL_LL_FRONT_DIST = 0; // meters, measure

        public static final double GOAL_HORIZONTAL_OFFSET = 0; // goal of x displacement from robot to ball/target - ideally 0 but if limelight not center change 
        public static final double BALL_THRESHOLD = 5;
        
        public static final double BALL_VISION_kF = 0.8;
        public static final double BALL_VISION_kP = 0.01;
        public static final double BALL_VISION_kD = 0.00001;
        public static final double BALL_AUTO_PURSUIT_kF = 0.4;

        public static final double BALL_DECELERATE_START_DISTANCE = 25 * ConversionConstants.INCHES_TO_METERS; 
        public static final double BALL_DECELERATE_END_DISTANCE = 9.5 * ConversionConstants.INCHES_TO_METERS; 

        public static final double BALL_VEL_THRESHOLD = 2.54; // m/s - 100 in/s 
        public static final int BALL_VEL_PLATEAU_THRESHOLD = 10;
    }
}
