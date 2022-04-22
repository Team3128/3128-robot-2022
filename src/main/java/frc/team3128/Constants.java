package frc.team3128;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import frc.team3128.common.utility.interpolation.InterpolatingDouble;
import frc.team3128.common.utility.interpolation.InterpolatingTreeMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

// CURRENTLY CONFIGURED FOR 4 FALCON DRIVE (Compbot)

public class Constants {

    public static class ConversionConstants {

        public static final double FALCON_ENCODER_RESOLUTION = 2048;
        public static final double SPARK_ENCODER_RESOLUTION = 42;
        public static final double SPARK_VELOCITY_FACTOR = SPARK_ENCODER_RESOLUTION / 60; // RPM to nu/s
        public static final double FALCON_NUp100MS_TO_RPM = 10 * 60 / FALCON_ENCODER_RESOLUTION; // sensor units per 100 ms to rpm
        public static final double FALCON_NUpS_TO_RPM = 60 / FALCON_ENCODER_RESOLUTION; // sensor units per second to rpm
    }

    public static class DriveConstants {

        public static final int DRIVE_MOTOR_LEFT_LEADER_ID = 0;
        public static final int DRIVE_MOTOR_LEFT_FOLLOWER_ID = 1;
        public static final int DRIVE_MOTOR_RIGHT_LEADER_ID = 2;
        public static final int DRIVE_MOTOR_RIGHT_FOLLOWER_ID = 3;

        public static final double ARCADE_DRIVE_TURN_MULT = 0.70 * 0.35;
        public static final double ARCADE_DRIVE_RATE_LIMIT = 2.0; //max rate of change in the forward parameter (joystick Y axis) given to arcade drive

        public static final double DRIVE_GEARING = 9.6;
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.029);
        public static final double TRACK_WIDTH_METERS = 0.56147;

        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
        public static final double ENCODER_DISTANCE_PER_MARK = WHEEL_RADIUS_METERS * 2 * Math.PI / ConversionConstants.FALCON_ENCODER_RESOLUTION;
        public static final double DRIVE_NU_TO_METER = ENCODER_DISTANCE_PER_MARK / DRIVE_GEARING; // meters driven per encoder tick
        public static final double DRIVE_NUp100MS_TO_MPS = DRIVE_NU_TO_METER * 10; // sensor units per 100 ms to m/s of drivetrain
        public static final double MAX_DRIVE_VEL_NUp100MS = 6380 * ConversionConstants.FALCON_ENCODER_RESOLUTION / 60 / 10; // max angular velocity of drivetrain (encoder, not wheel) in sensor units per 100 ms - 6380 RPM * RESOLUTION nu/rot * 1 min/60s * 1s/(10*100ms)

        public static final double kS = 0.73226;
        public static final double kV = 2.0859;
        public static final double kA = 0.37853;
        public static final double kVAngular = 1.5;       // Nathan's magic numbers of doom
        public static final double kAAngular = 0.3;     // Nathan's magic numbers of doom

        public static final double MAX_DRIVE_VELOCITY = 2.5; // m/s - Real value ~5
        public static final double MAX_DRIVE_ACCELERATION = 2; // m/s^2 - I don't know what this number is
        public static final double MAX_DRIVE_VOLTAGE = 7; // volts (hopefully you could figure this out)

        public static final double TURN_kP = -2e-3;
        public static final double TURN_kI = 0;
        public static final double TURN_kD = -2e-4;
        public static final double TURN_kF = 0.1;

        public static final double TURN_TOLERANCE = 12; // deg

        //Ramsete constants
        public static final double RAMSETE_B = 2; //default value - don't change unless absolutely necessary
        public static final double RAMSETE_ZETA = 0.7; //default value - don't change unless absolutely necessary
        public static final double RAMSETE_KP = 3.0101;

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

    public static class ShooterConstants {

        public static final double SET_RPM = 3000;
        public static final double SET_ANGLE = 25;

        public static final int LEFT_SHOOTER_ID = 4; 
        public static final int RIGHT_SHOOTER_ID = 5; 

        public static final double LOW_kP = 1e-4; //1.24e-6;
        public static final double LOW_kI = 0;
        public static final double LOW_kD = 0;

        public static final double HIGH_kP = 2e-4; // 3.2e-3;
        public static final double HIGH_kI = 0;
        public static final double HIGH_kD = 0; // 5e-4;

        public static final double LOW_kS = 0.2; // 0.711; //Static gain in PID Feed Forward
        public static final double LOW_kV = 0.0017857 * 0.9; // 0.00163; //Velocity gain in PID Feed Forward
        public static final double LOW_kA = 0.0053359 * 0.9; // 0.0349; //Acceleration gain PID Feed Forward

        public static final double HIGH_kS = 0.2;//* 0.925; //Static gain in PID Feed Forward
        public static final double HIGH_kV = 0.0017857;//* 0.97; //Velocity gain in PID Feed Forward
        public static final double HIGH_kA = 0.0053359;//* 0.99; //Acceleration gain PID Feed Forward

        public static final double kF = 0.00190678;

        public static final int PLATEAU_COUNT = 6;
        public static final double RPM_THRESHOLD_PERCENT = 0.05;
        public static final double RPM_THRESHOLD_PERCENT_MAX = 0.15;
        public static final double TIME_TO_MAX_THRESHOLD = 8;

        public static final LinearSystem<N1, N1, N1> SHOOTER_CHAR = 
        LinearSystemId.identifyVelocitySystem(
            LOW_kV, 
            LOW_kA
        );
        public static final double SHOOTER_RADIUS_METERS = 0.0508;
        public static final DCMotor SHOOTER_GEARBOX = DCMotor.getCIM(2);
        public static final double SHOOTER_GEARING = 1.5;

        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shooterSpeedsMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
        static {
            // shooterSpeedsMap.put(new InterpolatingDouble(188.0), new InterpolatingDouble(3750.0));
            // shooterSpeedsMap.put(new InterpolatingDouble(178.0), new InterpolatingDouble(3725.0));
            // shooterSpeedsMap.put(new InterpolatingDouble(168.0), new InterpolatingDouble(3650.0));
            // shooterSpeedsMap.put(new InterpolatingDouble(158.0), new InterpolatingDouble(3550.0));
            // shooterSpeedsMap.put(new InterpolatingDouble(148.0), new InterpolatingDouble(3000.0));
            // shooterSpeedsMap.put(new InterpolatingDouble(138.0), new InterpolatingDouble(3280.0));
            // shooterSpeedsMap.put(new InterpolatingDouble(128.0), new InterpolatingDouble(3100.0));
            // shooterSpeedsMap.put(new InterpolatingDouble(118.0), new InterpolatingDouble(3000.0));
            // shooterSpeedsMap.put(new InterpolatingDouble(108.0), new InterpolatingDouble(2870.0));

            shooterSpeedsMap.put(new InterpolatingDouble(210.0), new InterpolatingDouble(3675.0)); // fake inches :)
            shooterSpeedsMap.put(new InterpolatingDouble(190.0), new InterpolatingDouble(3460.0 + 125));
            shooterSpeedsMap.put(new InterpolatingDouble(180.0), new InterpolatingDouble(3360.0 + 125));
            shooterSpeedsMap.put(new InterpolatingDouble(170.0), new InterpolatingDouble(3300.0 + 125));
            shooterSpeedsMap.put(new InterpolatingDouble(160.0), new InterpolatingDouble(3150.0 + 150));
            shooterSpeedsMap.put(new InterpolatingDouble(150.0), new InterpolatingDouble(3050.0 + 100));
            shooterSpeedsMap.put(new InterpolatingDouble(140.0), new InterpolatingDouble(2950.0 + 200));
            shooterSpeedsMap.put(new InterpolatingDouble(130.0), new InterpolatingDouble(2900.0 + 200));
            shooterSpeedsMap.put(new InterpolatingDouble(120.0), new InterpolatingDouble(2800.0 + 100));
            shooterSpeedsMap.put(new InterpolatingDouble(110.0), new InterpolatingDouble(2750.0 + 100));

            shooterSpeedsMap.put(new InterpolatingDouble(98.0), new InterpolatingDouble(2750.0));
            shooterSpeedsMap.put(new InterpolatingDouble(88.0), new InterpolatingDouble(2700.0));
            shooterSpeedsMap.put(new InterpolatingDouble(78.0), new InterpolatingDouble(2650.0));
            shooterSpeedsMap.put(new InterpolatingDouble(68.0), new InterpolatingDouble(2625.0));
            shooterSpeedsMap.put(new InterpolatingDouble(58.0), new InterpolatingDouble(2600.0));
            shooterSpeedsMap.put(new InterpolatingDouble(48.0), new InterpolatingDouble(2550.0));
            shooterSpeedsMap.put(new InterpolatingDouble(38.0), new InterpolatingDouble(2500.0));
            shooterSpeedsMap.put(new InterpolatingDouble(28.0), new InterpolatingDouble(2375.0));
            
        };
    }

    public static class HopperConstants {

        public static final int HOPPER_MOTOR_ID = 6;
        public static final int HOPPER_MOTOR_2_ID = 9;
        public static final int BOTTOM_SENSOR_ID = 12;
        public static final int TOP_SENSOR_ID = 13;

        //public static final int HOPPER_SOLENOID_FORWARD_CHANNEL_ID = 5;
        //public static final int HOPPER_SOLENOID_BACKWARD_CHANNEL_ID = 6;

        public static final int HOPPER_DIO_PIN1 = 8;
        public static final int HOPPER_DIO_PIN2 = 9;
        public static final double HOPPER_MAX_REVERSE_DISTANCE = -2200; //set distance

        public static final double HOPPER_MOTOR_POWER = 0.6;
        public static final double HOPPER_MOTOR_2_POWER = 0.7;
        public static final double REVERSE_HOPPER_MOTOR_POWER = -1; //change this

    }

    public static class IntakeConstants {

        public static final int INTAKE_MOTOR_ID = 7; 
        public static final int INTAKE_SOLENOID_FORWARD_CHANNEL_ID = 4;
        public static final int INTAKE_SOLENOID_BACKWARD_CHANNEL_ID = 3;

        public static final double INTAKE_MOTOR_POWER = -1; //negative is forward 
        public static final double OUTTAKE_MOTOR_POWER = 1;

    }

    public static class ClimberConstants {

        public static final int CLIMBER_MOTOR_LEFT_ID = 12;
        public static final int CLIMBER_MOTOR_RIGHT_ID = 13;

        public static final int CLIMBER_SENSOR_LEFT_ID = 7;
        public static final int CLIMBER_SENSOR_RIGHT_ID = 6;

        public static final int CLIMBER_SOLENOID_FORWARD_CHANNEL_ID = 2; 
        public static final int CLIMBER_SOLENOID_BACKWARD_CHANNEL_ID = 6;
        public static final int CLIMBER_SOLENOID_BREAK_FORWARD_CHANNEL_ID = 0; //1
        public static final int CLIMBER_SOLENOID_BREAK_BACKWARD_CHANNEL_ID = 7; //5

        public static final double CLIMBER_GEAR_RATIO = 18.9;
        public static final double AXLE_DIAMETER = 0.7;
        public static final double CLIMBER_ERROR_RATE = .5; //in inches

        public static final double CLIMBER_HEIGHT = 20; // inches

        public static final double VERTICAL_DISTANCE = 23.875;  // TODO: change to actual distance
        public static final double SMALL_VERTICAL_DISTANCE = 5; // TODO: change to actual distance
        public static final double ANGLED_DISTANCE = 12; // TODO: change to actual distance
    
        public static final NeutralMode CLIMBER_NEUTRAL_MODE = NeutralMode.Brake;
        public static final double CLIMBER_POWER = 0.8; // 0.9
        public static final double MANUAL_POWER = 0.3;

        public static final double CLIMB_ENC_DIAG_EXTENSION = 322000; // 6600;//same as diag extension rn // old-7850; //-7379
        public static final double CLIMB_ENC_TO_TOP = 322000; // 6600 //  old -7825;
        public static final double TOLERANCE_TICKS = 1000; // 100

    }

    public static class HoodConstants {

        public static final int HOOD_MOTOR_ID = 5;

        public static final double kP = 0.3;//0.4;//0.3128;//0.254;
        public static final double kI = 0;
        public static final double kD = 0;//8e-6;
        public static final double kF = 0.49072349;//0.6134;//1.2268;//0.6134;//1.2268; //0.6134;//0.0517638;

        public static final int PLATEAU_COUNT = 3;
        public static final double TOLERANCE_MIN = 0.1; // degrees
        public static final double TOLERANCE_MAX = 0.5; // degrees
        public static final double TIME_TO_MAX_TOLERANCE = 4; // seconds

        public static final int HOOD_SHOOTER_GEAR_RATIO = 111; // Bowen number
        public static final double ENC_POSITION_CONVERSION_FACTOR = 360.0 / HOOD_SHOOTER_GEAR_RATIO; // Multiply by this to convert encoder rotations to hood degrees

        public static final int HOOD_CURRENT_LIMIT = 10; // Amps

        public static final double MIN_ANGLE = 3.0; // deg
        public static final double MAX_ANGLE = 34.0; // deg
        public static final double HOME_ANGLE = 22.0; // deg

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodAngleMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
        static {
            // hoodAngleMap.put(new InterpolatingDouble(178.0), new InterpolatingDouble(30.0));
            // hoodAngleMap.put(new InterpolatingDouble(168.0), new InterpolatingDouble(29.0));
            // hoodAngleMap.put(new InterpolatingDouble(158.0), new InterpolatingDouble(28.5));
            // hoodAngleMap.put(new InterpolatingDouble(148.0), new InterpolatingDouble(28.0));
            // hoodAngleMap.put(new InterpolatingDouble(138.0), new InterpolatingDouble(26.5));
            // hoodAngleMap.put(new InterpolatingDouble(128.0), new InterpolatingDouble(25.5));
            // hoodAngleMap.put(new InterpolatingDouble(118.0), new InterpolatingDouble(24.5));
            // hoodAngleMap.put(new InterpolatingDouble(108.0), new InterpolatingDouble(23.0));

            hoodAngleMap.put(new InterpolatingDouble(190.0), new InterpolatingDouble(32.5));
            hoodAngleMap.put(new InterpolatingDouble(180.0), new InterpolatingDouble(32.5));
            hoodAngleMap.put(new InterpolatingDouble(170.0), new InterpolatingDouble(32.5));
            hoodAngleMap.put(new InterpolatingDouble(160.0), new InterpolatingDouble(31.5));
            hoodAngleMap.put(new InterpolatingDouble(150.0), new InterpolatingDouble(27.0));
            hoodAngleMap.put(new InterpolatingDouble(140.0), new InterpolatingDouble(27.5));
            hoodAngleMap.put(new InterpolatingDouble(130.0), new InterpolatingDouble(26.5));
            hoodAngleMap.put(new InterpolatingDouble(120.0), new InterpolatingDouble(24.0));
            hoodAngleMap.put(new InterpolatingDouble(110.0), new InterpolatingDouble(23.5));

            hoodAngleMap.put(new InterpolatingDouble(98.0), new InterpolatingDouble(22.0));
            hoodAngleMap.put(new InterpolatingDouble(88.0), new InterpolatingDouble(19.0));
            hoodAngleMap.put(new InterpolatingDouble(78.0), new InterpolatingDouble(16.0));
            hoodAngleMap.put(new InterpolatingDouble(68.0), new InterpolatingDouble(14.5));
            hoodAngleMap.put(new InterpolatingDouble(58.0), new InterpolatingDouble(13.0));
            hoodAngleMap.put(new InterpolatingDouble(48.0), new InterpolatingDouble(12.0));
            hoodAngleMap.put(new InterpolatingDouble(38.0), new InterpolatingDouble(11.0));
            hoodAngleMap.put(new InterpolatingDouble(28.0), new InterpolatingDouble(9.0));
            
        };
    }

    public static class VisionConstants {

        public static final String TOP_HOSTNAME = "limelight-cog";

        public static final int SAMPLE_RATE = 3;

        public static final double TOP_CAMERA_ANGLE = (90 - 42.71) * Math.PI / 180; // radians
        public static final double TOP_CAMERA_HEIGHT = 23.5; // in 
        public static final double TOP_FRONT_DIST = 0;
        public static final double TARGET_HEIGHT = 104;

        public static final double VISION_PID_kP = 2.2e-3;
        public static final double VISION_PID_kI = 0; // 0.02;
        public static final double VISION_PID_kD = 0; // 0.00006;
        public static final double VISION_PID_kF = 0.06;

        public static final double TX_OFFSET = 0; // to offset alignment in either direction

        public static final double TX_THRESHOLD = 2; //degrees
        public static final double TX_THRESHOLD_MAX = 5; //degrees
        public static final double TIME_TO_MAX_THRESHOLD = 5; //seconds
        public static final double TX_THRESHOLD_INCREMENT = (TX_THRESHOLD_MAX - TX_THRESHOLD) / TIME_TO_MAX_THRESHOLD; //degrees per second

        public static final int ALIGN_PLATEAU_COUNT = 10; //Number of checks at correct RPM to shoot

        public static final double BALL_TARGET_HEIGHT = 9.5 / 2;
        public static final double BALL_LL_HEIGHT = 24;
        public static final double BALL_LL_ANGLE = 65.15 * Math.PI / 180; // 1.0; // Math.acos(21.0 / 39.0); // 1.002186; // radians
        public static final double BALL_LL_FRONT_DIST = 0; // meters, measure

        public static final double GOAL_HORIZONTAL_OFFSET = 0; // goal of x displacement from robot to ball/target - ideally 0 but if limelight not center change 
        public static final double BALL_THRESHOLD = 5;
        
        public static final double BALL_VISION_kF = 0.8;
        public static final double BALL_VISION_kP = 0.01;
        public static final double BALL_VISION_kD = 0.00001;
        public static final double BALL_AUTO_PURSUIT_kF = 0.4;

        public static final double BALL_DECELERATE_START_DISTANCE = 25; 
        public static final double BALL_DECELERATE_END_DISTANCE = 9.5; 

        public static final double BALL_VEL_THRESHOLD = 2.54; // m/s - 100 in/s 
        public static final int BALL_VEL_PLATEAU_THRESHOLD = 10;
    }
}
