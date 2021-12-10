package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Robot;
import frc.team3128.common.hardware.motor.NAR_TalonFX;

public class NAR_Drivetrain extends SubsystemBase {

    private static NAR_Drivetrain instance;
    
    private static BaseTalon leftLeader;
    private static BaseTalon rightLeader;
    private static NAR_TalonFX leftFollower;
    private static NAR_TalonFX rightFollower;
    private static TalonSRXSimCollection leftMotorSim;
    private static TalonSRXSimCollection rightMotorSim;
    private static DifferentialDrive robotDrive;
    private static DifferentialDrivetrainSim robotDriveSim;
    private static DifferentialDriveOdometry odometry;

    private static AHRS gyro = new AHRS(SPI.Port.kMXP);;

    private static Field2d field;

    public NAR_Drivetrain(){

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        if (Robot.isReal()) {
            leftLeader = new NAR_TalonFX(0);
            leftFollower = new NAR_TalonFX(1);
            rightLeader = new NAR_TalonFX(2);
            rightFollower = new NAR_TalonFX(3);

            leftFollower.follow(leftLeader);
            leftFollower.setInverted(InvertType.FollowMaster);
            rightFollower.follow(rightLeader);
            rightFollower.setInverted(InvertType.FollowMaster);

            robotDrive = new DifferentialDrive((NAR_TalonFX)leftLeader, (NAR_TalonFX)rightLeader);
        } 
        else {
            leftLeader = new WPI_TalonSRX(0);
            rightLeader = new WPI_TalonSRX(1);
            leftMotorSim = new TalonSRXSimCollection(leftLeader);
            rightMotorSim = new TalonSRXSimCollection(rightLeader);
            robotDrive = new DifferentialDrive((WPI_TalonSRX)leftLeader, (WPI_TalonSRX)rightLeader);

            // Simulated drivetrain handles simulation math
            robotDriveSim =
            new DifferentialDrivetrainSim(
                LinearSystemId.identifyDrivetrainSystem(
                    0.5, // kvVoltSecondsPerMeter
                    0.05, // kaVoltSecondsSquaredPerMeter
                    1.5, // kvVoltSecondsPerRadian
                    0.3 // kaVoltSecondsSquaredPerRadian
                ),
                DCMotor.getFalcon500(4), // gearbox
                8, // driveGearing
                0.66, // trackWidthMeters
                2, // wheelRadius
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
    }

    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the simulation,
        // and write the simulated positions and velocities to our simulated encoder and gyro.
        // We negate the right side so that positive voltages make the right side
        // move forward.
        robotDriveSim.setInputs(
            -leftLeader.getMotorOutputVoltage(),// * RobotController.getBatteryVoltage(),
            rightLeader.getMotorOutputVoltage());// * RobotController.getBatteryVoltage());

        robotDriveSim.update(0.02);

        leftMotorSim.setQuadratureRawPosition((int)(robotDriveSim.getLeftPositionMeters() / Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK));
        leftMotorSim.setQuadratureVelocity((int)(robotDriveSim.getLeftVelocityMetersPerSecond()/Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK/10));
        rightMotorSim.setQuadratureRawPosition((int)(robotDriveSim.getRightPositionMeters() / Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK));
        rightMotorSim.setQuadratureVelocity((int)(robotDriveSim.getRightVelocityMetersPerSecond()/Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK/10));

        SmartDashboard.putNumber("Left Speed", leftLeader.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Left Desired Speed", robotDriveSim.getLeftVelocityMetersPerSecond() / Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK * 10);
        
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(-robotDriveSim.getHeading().getDegrees());

        // leftMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        // rightMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    }
        
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getLeftEncoderDistance() {
        return leftLeader.getSelectedSensorPosition() * Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK;
    }

    public double getRightEncoderDistance() {
        return rightLeader.getSelectedSensorPosition() * Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK;
    }

    /**
     * @return the left encoder velocity in meters per second
     */
    public double getLeftEncoderSpeed() {
        return leftLeader.getSelectedSensorVelocity() * Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK * 10;
    }

    /**
     * @return the right encoder velocity in meters per second
     */
    public double getRightEncoderSpeed() {
        return rightLeader.getSelectedSensorVelocity() * Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK * 10;
    }

    public void arcadeDrive(double x, double y) {
        robotDrive.arcadeDrive(x, y, false); // Don't squareInputs
    }

    public void resetEncoders() {
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
    }

    public void stop() {
        robotDrive.stopMotor();
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        robotDrive.tankDrive(leftSpeed, rightSpeed);
    }
    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderSpeed(), getRightEncoderSpeed());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        robotDrive.tankDrive(leftVolts / RobotController.getBatteryVoltage(), rightVolts / RobotController.getBatteryVoltage());
    }
}

