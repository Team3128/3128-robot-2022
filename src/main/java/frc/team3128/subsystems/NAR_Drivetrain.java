package frc.team3128.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants;
import frc.team3128.Robot;
import frc.team3128.hardware.motor.*;
import frc.team3128.infrastructure.NAR_EMotor;

// CURRENTLY CONFIGURED FOR 4 FALCON DRIVE (Speedy G)

public class NAR_Drivetrain extends SubsystemBase {

    // Initialize the generic motors

    // private NAR_EMotor leftLeader = new NAR_TalonFX(Constants.DriveConstants.DRIVE_MOTOR_LEFT_LEADER_ID);
    // private NAR_EMotor rightLeader = new NAR_TalonFX(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_LEADER_ID);
    // private NAR_EMotor leftFollower = new NAR_TalonFX(Constants.DriveConstants.DRIVE_MOTOR_LEFT_FOLLOWER_ID);
    // private NAR_EMotor rightFollower = new NAR_TalonFX(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_FOLLOWER_ID);

    private NAR_EMotor leftLeader = new NAR_TalonSRX(Constants.DriveConstants.DRIVE_MOTOR_LEFT_LEADER_ID);
    private NAR_EMotor rightLeader = new NAR_TalonSRX(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_LEADER_ID);
    private NAR_EMotor leftFollower = new NAR_TalonSRX(Constants.DriveConstants.DRIVE_MOTOR_LEFT_FOLLOWER_ID);
    private NAR_EMotor rightFollower = new NAR_TalonSRX(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_FOLLOWER_ID);
    
    // private NAR_EMotor leftLeader = new NAR_CANSparkMax(Constants.DriveConstants.KIT_MOTOR_LEFT_LEADER_ID, MotorType.kBrushless);
    // private NAR_EMotor rightLeader = new NAR_CANSparkMax(Constants.DriveConstants.KIT_MOTOR_RIGHT_LEADER_ID, MotorType.kBrushless);
    // private NAR_EMotor leftFollower = new NAR_CANSparkMax(Constants.DriveConstants.KIT_MOTOR_LEFT_FOLLOWER_ID, MotorType.kBrushless);
    // private NAR_EMotor rightFollower = new NAR_CANSparkMax(Constants.DriveConstants.KIT_MOTOR_RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    public static NAR_Drivetrain instance;

    private DifferentialDrive robotDrive;
    private DifferentialDrivetrainSim robotDriveSim;
    private DifferentialDriveOdometry odometry;

    private static AHRS gyro = new AHRS(SPI.Port.kMXP);

    private static Field2d field;

    public NAR_Drivetrain(){

        leftFollower.follow(leftLeader);
        rightFollower.follow((rightLeader));
        
        // Right side true for kitbot, left side true for speedy G
        leftLeader.setInverted(true);
        leftFollower.setInverted(true);
        rightLeader.setInverted(false);
        rightFollower.setInverted(false);

        robotDrive = new DifferentialDrive(
            new MotorControllerGroup(leftLeader, leftFollower),
            new MotorControllerGroup(rightLeader, rightFollower));

        if(Robot.isSimulation()){
            robotDriveSim =
            new DifferentialDrivetrainSim(
                Constants.DriveConstants.DRIVE_CHAR,
                Constants.DriveConstants.GEARBOX,
                Constants.DriveConstants.DRIVE_GEARING,
                Constants.DriveConstants.TRACK_WIDTH_METERS,
                Constants.DriveConstants.WHEEL_RADIUS_METERS, 
                null/*VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005)*/);
        }

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        field = new Field2d();

        SmartDashboard.putData("Field", field);

        resetEncoders();
    }

    public static synchronized NAR_Drivetrain getInstance() {
        if (instance == null) {
            instance = new NAR_Drivetrain();
        }
        return instance;
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderDistance(), getRightEncoderDistance());
        field.setRobotPose(getPose());   
        
        SmartDashboard.putNumber("Left Encoder (meters)", getLeftEncoderDistance());
        SmartDashboard.putNumber("Right Encoder (meters)", getRightEncoderDistance());
        SmartDashboard.putNumber("Gyro", getHeading());
    }

    public void simulationPeriodic() {
        
        // Set motor voltage inputs
        robotDriveSim.setInputs(
            leftLeader.getMotorOutputVoltage(),
            rightLeader.getMotorOutputVoltage()
        );

        // Update sim environment
        robotDriveSim.update(0.02);

        // Store simulated motor states
        leftLeader.setSimPosition(robotDriveSim.getLeftPositionMeters() / Constants.DriveConstants.DRIVE_DIST_PER_TICK);
        leftLeader.setSimVelocity(robotDriveSim.getLeftVelocityMetersPerSecond() / Constants.DriveConstants.DRIVE_DIST_PER_TICK);
        rightLeader.setSimPosition(robotDriveSim.getRightPositionMeters() / Constants.DriveConstants.DRIVE_DIST_PER_TICK);
        rightLeader.setSimVelocity(robotDriveSim.getRightVelocityMetersPerSecond() / Constants.DriveConstants.DRIVE_DIST_PER_TICK);

        SmartDashboard.putNumber("Left Sim Speed", leftLeader.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Right Sim Speed", rightLeader.getSelectedSensorVelocity());
        
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(robotDriveSim.getHeading().getDegrees()); // @Nathan: I tested this out, this seems to work. This preserves parity w/ the real robot in angle, odometry
    }
        
    public double getHeading() {
        //gyro.getYaw uses CW as positive
        return -gyro.getYaw();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getLeftEncoderDistance() {
        return leftLeader.getSelectedSensorPosition() * Constants.DriveConstants.DRIVE_DIST_PER_TICK;
    }

    public double getRightEncoderDistance() {
        return rightLeader.getSelectedSensorPosition() * Constants.DriveConstants.DRIVE_DIST_PER_TICK;
    }

    /**
     * @return the left encoder velocity in meters per second
     */
    public double getLeftEncoderSpeed() {
        return leftLeader.getSelectedSensorVelocity() * Constants.DriveConstants.DRIVE_DIST_PER_TICK * 10;
    }

    /**
     * @return the right encoder velocity in meters per second
     */
    public double getRightEncoderSpeed() {
        return rightLeader.getSelectedSensorVelocity() * Constants.DriveConstants.DRIVE_DIST_PER_TICK * 10;
    }
    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderSpeed(), getRightEncoderSpeed());
    }

    public void arcadeDrive(double x, double y) {
        robotDrive.arcadeDrive(x, y, false);
    }

    public void stop() {
        robotDrive.stopMotor();
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        robotDrive.tankDrive(leftSpeed, rightSpeed);
        robotDrive.feed();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        robotDrive.tankDrive(leftVolts / RobotController.getBatteryVoltage(), rightVolts / RobotController.getBatteryVoltage());
        robotDrive.feed();
    }

    public void resetEncoders() {
        leftLeader.setEncoderPosition(0);
        rightLeader.setEncoderPosition(0);
    }

    public void resetPose(Pose2d poseMeters) {
        resetEncoders();
        odometry.resetPosition(poseMeters, Rotation2d.fromDegrees(getHeading()));
    }

    public void resetGyro() {
        gyro.reset();
    }

}

