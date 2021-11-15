package frc.team3128.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.team3128.hardware.*;
import frc.team3128.Robot;

public class GoodDriveTrain extends SubsystemBase {
    
    private static BaseTalon leftMotor;
    private static BaseTalon rightMotor;
    private static TalonSRXSimCollection leftMotorSim;
    private static TalonSRXSimCollection rightMotorSim;
    private static DifferentialDrive robotDrive;
    private static DifferentialDrivetrainSim robotDriveSim;
    private static DifferentialDriveOdometry odometry;

    private static AHRS gyro = new AHRS(SPI.Port.kMXP);;

    private static Field2d field;

    private static double encoderDistancePerDot = 

    2 * 2 // wheel diameter
    * Math.PI
    / 2048; // encoder resolution

    public GoodDriveTrain(){

      odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

      if(Robot.isReal()){
          leftMotor = new GoodTalonFX(0);
          rightMotor = new GoodTalonFX(1);
          robotDrive = new DifferentialDrive((GoodTalonFX)leftMotor, (GoodTalonFX)rightMotor);
        }else{
          leftMotor = new WPI_TalonSRX(0);
          rightMotor = new WPI_TalonSRX(1);
          robotDrive = new DifferentialDrive((WPI_TalonSRX)leftMotor, (WPI_TalonSRX)rightMotor);
          leftMotorSim = new TalonSRXSimCollection(leftMotor);
          rightMotorSim = new TalonSRXSimCollection(rightMotor);

          field = new Field2d();

          SmartDashboard.putData("Field", field);

          //SmartDashboard.putData("Field", m_field);

          // This class simulates our drivetrain's motion around the field.
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

        resetEncoders();
    }

  @Override
  public void periodic() {
    odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        getLeftEncoderDistance(),
        getRightEncoderDistance());
    field.setRobotPose(getPose());   
  }

  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    robotDriveSim.setInputs(
        -leftMotor.getMotorOutputVoltage(),// * RobotController.getBatteryVoltage(),
        rightMotor.getMotorOutputVoltage());// * RobotController.getBatteryVoltage());

    robotDriveSim.update(0.02);

    leftMotorSim.setQuadratureRawPosition((int)(robotDriveSim.getLeftPositionMeters() / encoderDistancePerDot));
    leftMotorSim.setQuadratureVelocity((int)(robotDriveSim.getLeftVelocityMetersPerSecond()/encoderDistancePerDot/10));
    rightMotorSim.setQuadratureRawPosition((int)(robotDriveSim.getRightPositionMeters() / encoderDistancePerDot));
    rightMotorSim.setQuadratureVelocity((int)(robotDriveSim.getRightVelocityMetersPerSecond()/encoderDistancePerDot/10));

    SmartDashboard.putNumber("Left Speed", leftMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Left Desired Speed", robotDriveSim.getLeftVelocityMetersPerSecond() / encoderDistancePerDot * 10);
    
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-robotDriveSim.getHeading().getDegrees());

    // leftMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    // rightMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
  }
    
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (false // gyroRversed
    ? -1.0 : 1.0);
    //return 0;
  }

  public Pose2d getPose(){

    return odometry.getPoseMeters();
  }

  public double getLeftEncoderDistance() {
    return leftMotor.getSelectedSensorPosition() * encoderDistancePerDot;
  }

  public double getRightEncoderDistance() {
    return rightMotor.getSelectedSensorPosition() * encoderDistancePerDot;
  }

  public void arcadeDrive(double x, double y){
    robotDrive.arcadeDrive(x,y);
  }

  public void resetEncoders() {
    leftMotor.setSelectedSensorPosition(0);
    rightMotor.setSelectedSensorPosition(0);
  }
}
