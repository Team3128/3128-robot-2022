package frc.team3128.common.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.common.swerve.FalconConversions.*;

import static frc.team3128.common.swerve.CTREConfigs.SwerveConfigs.*;

public class SwerveModule {
    public final int moduleNumber;
    private final double angleOffset;
    private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    private final CANCoder angleEncoder;

    // private final PIDController drivePIDController = new PIDController(driveKP, driveKI, driveKD);
    // private final PIDController anglePIDController = new PIDController(angleKP, angleKI, angleKD);
    private double lastAngle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        double velocity = MPSToFalcon(desiredState.speedMetersPerSecond, wheelCircumference, driveGearRatio);
        driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleMotor.set(ControlMode.Position, degreesToFalcon(angle, angleGearRatio)); 
        lastAngle = angle;
    }

    private void resetToAbsolute(){
        double absolutePosition = degreesToFalcon(getCanCoder().getDegrees() - angleOffset, angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(swerveAngleMotorConfig);
        angleMotor.setInverted(angleMotorInvert);
        angleMotor.setNeutralMode(NeutralMode.Coast);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(swerveDriveMotorConfig);
        driveMotor.setInverted(driveMotorInvert);
        driveMotor.setNeutralMode(NeutralMode.Coast);
        driveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState(){
        double velocity = falconToMPS(driveMotor.getSelectedSensorVelocity(), wheelCircumference, driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(falconToDegrees(angleMotor.getSelectedSensorPosition(), angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        angleMotor.set(ControlMode.PercentOutput, 0);
    }
    
}