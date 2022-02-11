package frc.team3128.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants.DriveConstants;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import frc.team3128.common.infrastructure.NAR_EMotor;

public class NAR_Drivetrain extends SubsystemBase {

    // Initialize the generic motors

    private NAR_EMotor leftLeader = new NAR_TalonFX(DriveConstants.DRIVE_MOTOR_LEFT_LEADER_ID);
    private NAR_EMotor rightLeader = new NAR_TalonFX(DriveConstants.DRIVE_MOTOR_RIGHT_LEADER_ID);
    private NAR_EMotor leftFollower = new NAR_TalonFX(DriveConstants.DRIVE_MOTOR_LEFT_FOLLOWER_ID);
    private NAR_EMotor rightFollower = new NAR_TalonFX(DriveConstants.DRIVE_MOTOR_RIGHT_FOLLOWER_ID);

    // private NAR_EMotor leftLeader = new NAR_TalonSRX(DriveConstants.DRIVE_MOTOR_LEFT_LEADER_ID);
    // private NAR_EMotor rightLeader = new NAR_TalonSRX(DriveConstants.DRIVE_MOTOR_RIGHT_LEADER_ID);
    // private NAR_EMotor leftFollower = new NAR_TalonSRX(DriveConstants.DRIVE_MOTOR_LEFT_FOLLOWER_ID);
    // private NAR_EMotor rightFollower = new NAR_TalonSRX(DriveConstants.DRIVE_MOTOR_RIGHT_FOLLOWER_ID);
    
    // private NAR_EMotor leftLeader = new NAR_CANSparkMax(DriveConstants.KIT_MOTOR_LEFT_LEADER_ID, MotorType.kBrushless);
    // private NAR_EMotor rightLeader = new NAR_CANSparkMax(DriveConstants.KIT_MOTOR_RIGHT_LEADER_ID, MotorType.kBrushless);
    // private NAR_EMotor leftFollower = new NAR_CANSparkMax(DriveConstants.KIT_MOTOR_LEFT_FOLLOWER_ID, MotorType.kBrushless);
    // private NAR_EMotor rightFollower = new NAR_CANSparkMax(DriveConstants.KIT_MOTOR_RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    private static NAR_Drivetrain instance;

    private DifferentialDrive robotDrive;
    private DifferentialDrivetrainSim robotDriveSim;
    private DifferentialDriveOdometry odometry;

    private static AHRS gyro = new AHRS(SPI.Port.kMXP);;

    private static Field2d field;

    public NAR_Drivetrain(){

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        
        leftLeader.setInverted(true);
        leftFollower.setInverted(true);
        rightLeader.setInverted(false);
        rightFollower.setInverted(false);

        robotDrive = new DifferentialDrive(
            new MotorControllerGroup(leftLeader, leftFollower),
            new MotorControllerGroup(rightLeader, rightFollower));

        if(RobotBase.isSimulation()){
            robotDriveSim = new DifferentialDrivetrainSim(
                DriveConstants.DRIVE_CHAR,
                DriveConstants.GEARBOX,
                DriveConstants.DRIVE_GEARING,
                DriveConstants.TRACK_WIDTH_METERS,
                DriveConstants.WHEEL_RADIUS_METERS, 
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
        SmartDashboard.putNumber("Left Encoder Speed (m per s)", getLeftEncoderSpeed());
        SmartDashboard.putNumber("Right Encoder (m per s)", getRightEncoderSpeed());
        SmartDashboard.putString("getPose()", getPose().toString());
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
        leftLeader.setSimPosition(robotDriveSim.getLeftPositionMeters() / DriveConstants.DRIVE_NU_TO_METER);
        leftLeader.setSimVelocity(robotDriveSim.getLeftVelocityMetersPerSecond() / DriveConstants.DRIVE_NU_TO_METER);
        rightLeader.setSimPosition(robotDriveSim.getRightPositionMeters() / DriveConstants.DRIVE_NU_TO_METER);
        rightLeader.setSimVelocity(robotDriveSim.getRightVelocityMetersPerSecond() / DriveConstants.DRIVE_NU_TO_METER);
        
        SmartDashboard.putNumber("Left Sim Speed", leftLeader.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Right Sim Speed", rightLeader.getSelectedSensorVelocity());

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(robotDriveSim.getHeading().getDegrees()); // @Nathan: I tested this out, this seems to work. This preserves parity w/ the real robot in angle, odometry
        SmartDashboard.putNumber("Sim Gyro", angle.get());
    }
        
    public double getHeading() {
        //gyro.getYaw uses CW as positive
        return -gyro.getYaw(); // (Math.IEEEremainder(gyro.getAngle(), 360) + 360) % 360;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getLeftEncoderDistance() {
        return leftLeader.getSelectedSensorPosition() * DriveConstants.DRIVE_NU_TO_METER;
    }

    public double getRightEncoderDistance() {
        return rightLeader.getSelectedSensorPosition() * DriveConstants.DRIVE_NU_TO_METER;
    }

    /**
     * @return the left encoder velocity in meters per second
     */
    public double getLeftEncoderSpeed() {
        return leftLeader.getSelectedSensorVelocity() * DriveConstants.DRIVE_NU_TO_METER;
    }

    /**
     * @return the right encoder velocity in meters per second
     */
    public double getRightEncoderSpeed() {
        return rightLeader.getSelectedSensorVelocity() * DriveConstants.DRIVE_NU_TO_METER;
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

    /**
     * @param leftSpeed the left speed on [-1.0, 1.0]
     * @param rightSpeed the right speed on [-1.0, 1.0]
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        robotDrive.tankDrive(leftSpeed, rightSpeed, false);
        robotDrive.feed();
    }

    /**
     * @param leftVolts Left-side voltage on [-12.0, 12.0]
     * @param rightVolts Right-side voltage on [-12.0, 12.0]
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        tankDrive(leftVolts / RobotController.getBatteryVoltage(), rightVolts / RobotController.getBatteryVoltage());
        robotDrive.feed();
    }

    /**
     * @param leftVel Left side speed in native encoder units per 100ms
     * @param rightVel right side speed in native encoder units per 100ms
     */
    public void setVelocity(double leftVel, double rightVel) {
        // leftLeader.set(ControlMode.Velocity, leftVel);
        // rightLeader.set(ControlMode.Velocity, rightVel);
        // robotDrive.feed();

        tankDrive(leftVel / DriveConstants.MAX_DRIVE_VEL_NUp100MS, rightVel / DriveConstants.MAX_DRIVE_VEL_NUp100MS);
    }

    /**
     * @param leftVelMpS left side speed in meters per second
     * @param rightVelMps right side speed in meters per second
     */
    public void setVelocityMpS(double leftVelMpS, double rightVelMps) {
        setVelocity(leftVelMpS / DriveConstants.DRIVE_NUp100MS_TO_MPS, rightVelMps / DriveConstants.DRIVE_NUp100MS_TO_MPS);
    }

    public void resetEncoders() {
        leftLeader.setEncoderPosition(0);
        rightLeader.setEncoderPosition(0);
    }

    public void resetPose(Pose2d poseMeters) {
        resetEncoders();
        odometry.resetPosition(poseMeters, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Reset pose to (x = 0, y = 0, theta = 0)
     */
    public void resetPose() {
        resetGyro();
        resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(getHeading())));
    }

    public void resetGyro() {
        gyro.reset();
    }

}

