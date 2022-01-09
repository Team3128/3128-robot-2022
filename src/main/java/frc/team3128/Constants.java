package frc.team3128;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N2;

public class Constants {

    public static class ConversionConstants {
        public static final double FALCON_ENCODER_RESOLUTION = 2048;
        public static final double SPARK_ENCODER_RESOLUTION = 42;
        public static final double FALCON_NUp100MS_TO_RPM = 10 * 60 / FALCON_ENCODER_RESOLUTION; // (sensor units per 100 ms to rpm)

    }

    public static class DriveConstants {

        public static final int DRIVE_MOTOR_LEFT_LEADER_ID = 0;
        public static final int DRIVE_MOTOR_LEFT_FOLLOWER_ID = 1;
        public static final int DRIVE_MOTOR_RIGHT_LEADER_ID = 2;
        public static final int DRIVE_MOTOR_RIGHT_FOLLOWER_ID = 3;

        // Sim constants, TODO: move to new class

        // TODO: Get actual kv, ka
        public static final DCMotor GEARBOX = DCMotor.getFalcon500(4); 
        public static final LinearSystem<N2, N2, N2> DRIVE_CHAR = 
        LinearSystemId.identifyDrivetrainSystem(
            5,              // kvVoltSecondsPerMeter
            0.5,            // kaVoltSecondsSquaredPerMeter
            5,              // kvVoltSecondsPerRadian
            0.5             // kaVoltSecondsSquaredPerRadian
        );
        public static final double DRIVE_GEARING = 8;
        public static final double WHEEL_RADIUS_METERS = 0.0508; 
        public static final double TRACK_WIDTH_METERS = 0.66;        
        public static final double ENCODER_DISTANCE_PER_MARK = WHEEL_RADIUS_METERS * 2 / ConversionConstants.FALCON_ENCODER_RESOLUTION;

        public static final Boolean GYRO_REVERSED = false;
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
