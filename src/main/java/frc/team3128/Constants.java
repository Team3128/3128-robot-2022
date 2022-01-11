package frc.team3128;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;

public class Constants {

    public static class ConversionConstants {
        public static final double FALCON_ENCODER_RESOLUTION = 2048;
        public static final double SPARK_ENCODER_RESOLUTION = 42;
        public static final double FALCON_NUp100MS_TO_RPM = 10 * 60 / FALCON_ENCODER_RESOLUTION; // (sensor units per 100 ms to rpm)
    }

    // Speedy Gonzales constants
    public static class DriveConstants {

        public static final int DRIVE_MOTOR_LEFT_LEADER_ID = 0;
        public static final int DRIVE_MOTOR_LEFT_FOLLOWER_ID = 1;
        public static final int DRIVE_MOTOR_RIGHT_LEADER_ID = 2;
        public static final int DRIVE_MOTOR_RIGHT_FOLLOWER_ID = 3;

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
        public static final double MAX_DRIVE_ACCELERATION = 100; // m/s^2 - I don't know what this number is
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
