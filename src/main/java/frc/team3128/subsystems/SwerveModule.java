// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;

public class SwerveModule {

  private final NAR_TalonFX m_driveMotor;
  private final NAR_TalonFX m_turningMotor;
  
  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              SwerveConstants.kMaxAngularSpeed, SwerveConstants.kModuleMaxAngularAcceleration));
  
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);
  private final CANCoder m_absoluteEncoder;
  private final double m_absoluteEncoderOffset;
  private final boolean m_absoluteEncoderReversed;

  private final SimpleMotorFeedforward m_driveFeedforward;
  private final SimpleMotorFeedforward m_turnFeedforward;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorID Motor ID for the drive motor.
   * @param turningMotorID Motor ID for the turning motor.
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   * 
   */
  public SwerveModule(int driveMotorID, int turningMotorID, 
        boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderID, boolean absoluteEncoderReversed,double absoluteEncoderOffset) {
    m_driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.kS,SwerveConstants.kV,SwerveConstants.kA); //1 ,3
    m_turnFeedforward = new SimpleMotorFeedforward(SwerveConstants.kS,SwerveConstants.kV, SwerveConstants.kA); //1 ,.5
    m_driveMotor = new NAR_TalonFX(driveMotorID);
    m_turningMotor = new NAR_TalonFX(turningMotorID);
    
    m_driveMotor.setInverted(driveMotorReversed);
    m_turningMotor.setInverted(turningMotorReversed);
    
    m_absoluteEncoder = new CANCoder(absoluteEncoderID);
    m_absoluteEncoderOffset = absoluteEncoderOffset;
    m_absoluteEncoderReversed = absoluteEncoderReversed;
    // m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    // m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_driveMotor.setDistancePerPulse(2 * Math.PI * 
    //     SwerveConstants.TRACK_WIDTH_METERS / ConversionConstants.FALCON_ENCODER_RESOLUTION);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / ConversionConstants.FALCON_ENCODER_RESOLUTION);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    resetEncoders();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    if(Math.abs(desiredState.speedMetersPerSecond) < .001){
        stop();
        return;
    }
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, getState().angle);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getTurningPosition(), state.angle.getDegrees());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }

  public double getDrivePosition(){
      return m_driveMotor.getSelectedSensorPosition() * SwerveConstants.DRIVE_NU_TO_METER;
  }

  public double getTurningPosition(){
      return m_turningMotor.getSelectedSensorPosition() * SwerveConstants.DRIVE_NU_TO_METER;
  }

  public double getDriveVelocity(){
      return m_driveMotor.getSelectedSensorVelocity() * SwerveConstants.DRIVE_NU_TO_METER;
  }

  public double getTurningVelocity(){
      return m_turningMotor.getSelectedSensorVelocity() * SwerveConstants.DRIVE_NU_TO_METER;
  }

  public double getAbsoluteEncoderRad(){
      double angle = m_absoluteEncoder.getBusVoltage()/ RobotController.getVoltage5V();
      angle *= 2.0 * Math.PI;
      angle -= m_absoluteEncoderOffset;
      return angle * (m_absoluteEncoderReversed ? -1.0: 1.0);
  }

  public void resetEncoders(){
      m_driveMotor.setEncoderPosition(0);
      m_turningMotor.setEncoderPosition(getAbsoluteEncoderRad());
  }

  public void stop() {
      m_driveMotor.setVoltage(0);
      m_turningMotor.setVoltage(0);
  }
}
