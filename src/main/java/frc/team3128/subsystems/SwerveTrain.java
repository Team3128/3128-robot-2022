// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants.SwerveConstants;

/** Represents a swerve drive style drivetrain. */
public class SwerveTrain extends SubsystemBase {

  private final SwerveModule m_frontLeft = new SwerveModule(SwerveConstants.kFrontLeftDriveMotorID,
                                                            SwerveConstants.kFrontLeftTurnMotorID,
                                                            SwerveConstants.kFrontLeftDriveReversed,
                                                            SwerveConstants.kFrontLeftTurnReversed,
                                                            SwerveConstants.kFrontLeftAbsoluteEncoderID,
                                                            SwerveConstants.kFrontLeftAbsoluteEncoderReversed,
                                                            SwerveConstants.kFrontLeftAbsoluteEncoderOffset);

  private final SwerveModule m_frontRight = new SwerveModule(SwerveConstants.kFrontRightDriveMotorID,
                                                            SwerveConstants.kFrontRightTurnMotorID,
                                                            SwerveConstants.kFrontRightDriveReversed,
                                                            SwerveConstants.kFrontRightTurnReversed,
                                                            SwerveConstants.kFrontRightAbsoluteEncoderID,
                                                            SwerveConstants.kFrontRightAbsoluteEncoderReversed,
                                                            SwerveConstants.kFrontRightAbsoluteEncoderOffset);

  private final SwerveModule m_backLeft = new SwerveModule(SwerveConstants.kBackLeftDriveMotorID,
                                                            SwerveConstants.kBackLeftTurnMotorID,
                                                            SwerveConstants.kBackLeftDriveReversed,
                                                            SwerveConstants.kBackLeftTurnReversed,
                                                            SwerveConstants.kBackLeftAbsoluteEncoderID,
                                                            SwerveConstants.kBackLeftAbsoluteEncoderReversed,
                                                            SwerveConstants.kBackLeftAbsoluteEncoderOffset);

  private final SwerveModule m_backRight = new SwerveModule(SwerveConstants.kBackRightDriveMotorID,
                                                            SwerveConstants.kBackRightTurnMotorID,
                                                            SwerveConstants.kBackRightDriveReversed,
                                                            SwerveConstants.kBackRightTurnReversed,
                                                            SwerveConstants.kBackRightAbsoluteEncoderID,
                                                            SwerveConstants.kBackRightAbsoluteEncoderReversed,
                                                            SwerveConstants.kBackRightAbsoluteEncoderOffset);

  private static WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(SwerveConstants.PigeonID);
  private final SwerveDriveKinematics m_kinematics = SwerveConstants.kDriveKinematics;
  public static SwerveTrain instance;
  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

  public SwerveTrain() {
    new Thread(() -> {
        try{
            Thread.sleep(1000);
            zeroHeading();
        } catch(Exception e){}
    }).start();
    resetEncoders();
  }

  public static synchronized SwerveTrain getInstance(){
      if(instance == null){
        instance = new SwerveTrain();
      }
      return instance;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    setModuleStates(swerveModuleStates);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,SwerveConstants.kMaxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }
  
  public void zeroHeading(){
      m_gyro.reset();
  }

  public double getHeading(){
      return Math.IEEEremainder(m_gyro.getAngle(),360);
  }

  public Rotation2d getRotation2d() {
      return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Gyro", getHeading());
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }
  
  public void resetPose(){
    zeroHeading();
    resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(getHeading())));
  }

  public void resetPose(Pose2d poseMeters) {
    resetEncoders();
    m_odometry.resetPosition(poseMeters, Rotation2d.fromDegrees(getHeading()));
}

  public void resetEncoders(){
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backLeft.resetEncoders();
    m_backRight.resetEncoders();
  }

  public void stop(){
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public void stopModules(){
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }
}
