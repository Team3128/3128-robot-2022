package frc.team3128.autonomous;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Constants;
import static frc.team3128.Constants.DriveConstants.*;
import frc.team3128.subsystems.NAR_Drivetrain;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
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

    public static Trajectory driveBack30In = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(Units.inchesToMeters(-30), 0, new Rotation2d(0)),
        backwardsTrajConfig);

    public static Trajectory twoBallTraj = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(Units.inchesToMeters(40), 0, new Rotation2d(0)),
        forwardTrajConfig);

    public static Trajectory driveForwards500In = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(Units.inchesToMeters(250), 0, new Rotation2d(0)),
        forwardTrajConfig);

    private static HashMap<String, Trajectory> trajectories = new HashMap<String, Trajectory>();

    static{
        final String[] trajectoryNames = {
            "S2H2_i",
            "S2H2_ii",
            "S2H1",
            "S2H2_iii",
            "S2H2_iv",
            "4Ball_Terminal180_i",
            "4Ball_Terminal180_ii",
            "Terminal2Tarmac",
            "Tarmac2Terminal",
            "Billiards_i",
            "Billiards_ii",
            "3Ballv2_i",
            "3Ballv2_ii",
            "5Ballv2_i",
            "5Ballv2_ii",
            "S1H1_i",
            "S1H1_ii",
            "S1H2_ii",
            "S1H2_iii"
        };
        for (String trajectoryName : trajectoryNames) {
            Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName + ".wpilib.json");
            try {
                trajectories.put(trajectoryName, TrajectoryUtil.fromPathweaverJson(path));
            } catch (IOException ex) {
                DriverStation.reportError("IOException loading trajectory " + trajectoryName, true);
            }
        }
        trajectories.put("driveBack30In", driveBack30In);
        trajectories.put("twoBallTraj", twoBallTraj);
        trajectories.put("driveForwards500In", driveForwards500In);
    }

    public static Trajectory get(String name) {
        return trajectories.get(name);
    }
    
    public static Command trajectoryCmd(String traj) {
        NAR_Drivetrain drive = NAR_Drivetrain.getInstance();
        return new RamseteCommand(
                        trajectories.get(traj),
                        drive::getPose,
                        new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                        new SimpleMotorFeedforward(kS, kV, kA),
                        DRIVE_KINEMATICS,
                        drive::getWheelSpeeds,
                        new PIDController(RAMSETE_KP, 0, 0),
                        new PIDController(RAMSETE_KP, 0, 0),
                        drive::tankDriveVolts,
                        drive)
                .andThen(() -> drive.stop());
    }
}
