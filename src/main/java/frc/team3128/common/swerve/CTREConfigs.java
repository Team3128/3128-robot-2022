package frc.team3128.common.swerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import static frc.team3128.Constants.SwerveConstants.*;

public class CTREConfigs {
    // TODO: check this is correctly written java lol
    public static class SwerveConfigs {
        public static TalonFXConfiguration swerveAngleMotorConfig = new TalonFXConfiguration();
        public static TalonFXConfiguration swerveDriveMotorConfig = new TalonFXConfiguration();
        public static CANCoderConfiguration swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        public static SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            angleEnableCurrentLimit, 
            angleContinuousCurrentLimit, 
            anglePeakCurrentLimit, 
            anglePeakCurrentDuration);

        static {
            swerveAngleMotorConfig.slot0.kP = angleKP;
            swerveAngleMotorConfig.slot0.kI = angleKI;
            swerveAngleMotorConfig.slot0.kD = angleKD;
            swerveAngleMotorConfig.slot0.kF = angleKF;
            swerveAngleMotorConfig.supplyCurrLimit = angleSupplyLimit;
            swerveAngleMotorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        }

        /* Swerve Drive Motor Configuration */
        public static SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            driveEnableCurrentLimit, 
            driveContinuousCurrentLimit, 
            drivePeakCurrentLimit, 
            drivePeakCurrentDuration);

        static {
            swerveDriveMotorConfig.slot0.kP = driveKP;
            swerveDriveMotorConfig.slot0.kI = driveKI;
            swerveDriveMotorConfig.slot0.kD = driveKD;
            swerveDriveMotorConfig.slot0.kF = driveKF;        
            swerveDriveMotorConfig.supplyCurrLimit = driveSupplyLimit;
            swerveDriveMotorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
            swerveDriveMotorConfig.closedloopRamp = closedLoopRamp;
        }

        
        /* Swerve CANCoder Configuration */
        static {
            swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            swerveCanCoderConfig.sensorDirection = canCoderInvert;
            swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
            swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        }
    }

}