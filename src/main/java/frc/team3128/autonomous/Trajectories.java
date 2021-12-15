package frc.team3128.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.team3128.subsystems.Constants;

/**
 * 
 * Store trajectories for autonomous. Edit points here. 
 * There are many magic numbers in this class - makes it slightly easier to edit trajectories. With more accurate odometry, we could avoid this.
 * 
 */
public class Trajectories {
    private static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV, Constants.DriveConstants.kA),
            Constants.DriveConstants.DRIVE_KINEMATICS, Constants.DriveConstants.MAX_DRIVE_VOLTAGE);

    private static final TrajectoryConfig forwardTrajConfig = new TrajectoryConfig(Constants.DriveConstants.MAX_DRIVE_VELOCITY,
            Constants.DriveConstants.MAX_DRIVE_ACCELERATION)
            .setKinematics(Constants.DriveConstants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint)
            .setReversed(false);

    private static final TrajectoryConfig backwardsTrajConfig = new TrajectoryConfig(Constants.DriveConstants.MAX_DRIVE_VELOCITY,
            Constants.DriveConstants.MAX_DRIVE_ACCELERATION)
            .setKinematics(Constants.DriveConstants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint)
            .setReversed(true);

    public static Trajectory trajectorySimple = TrajectoryGenerator.generateTrajectory(
        new Pose2d(7, 4, new Rotation2d(0)),
        List.of(),
        new Pose2d(14, 8, new Rotation2d(0)),
        forwardTrajConfig);

    public static Trajectory trajectoryLessSimple = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(Units.inchesToMeters(6), Units.inchesToMeters(-14))),
        new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(-15), new Rotation2d(0)),
        forwardTrajConfig);
}
