package frc.team3128.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.team3128.Constants.DriveConstants.*;
import frc.team3128.commands.CmdAlign;
import frc.team3128.commands.CmdExtendIntake;
import frc.team3128.commands.CmdExtendIntakeAndRun;
import frc.team3128.commands.CmdInPlaceTurn;
import frc.team3128.commands.CmdRetractHopper;
import frc.team3128.commands.CmdShoot;
import frc.team3128.commands.CmdShootAlign;
import frc.team3128.commands.CmdOuttake;
import frc.team3128.commands.CmdShootSingleBall;
import frc.team3128.commands.CmdShootTurnVision;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;
import frc.team3128.subsystems.Shooter;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang
 */

public class AutoPrograms {

    private NAR_Drivetrain drive;
    private Shooter shooter;
    private Intake intake;
    private Hopper hopper;
    private Hood hood;
    private LimelightSubsystem limelights;

    private HashMap<String, Trajectory> trajectories;

    public AutoPrograms() {
        super();
        drive = NAR_Drivetrain.getInstance();
        shooter = Shooter.getInstance();
        intake = Intake.getInstance();
        hopper = Hopper.getInstance();
        hood = Hood.getInstance();
        limelights = LimelightSubsystem.getInstance();

        loadTrajectories();
        initAutoSelector();
    }

    private void loadTrajectories() {
        trajectories = new HashMap<String, Trajectory>();
        
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
    }

    private void initAutoSelector() {
        String[] autoStrings = new String[] {"1 Ball", "2 Ball", "3 Ball", "4 Ball", "5 Ball", "S2H1", "S2H2", "S1H1", "S1I1", "S1H2", "Billiards"};
        NarwhalDashboard.addAutos(autoStrings);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getSelectedAutoName();
        // String selectedAutoName = "3 Ball"; // uncomment and change this for testing without opening Narwhal Dashboard

        if (selectedAutoName == null) {
            return null;
        }

        Pose2d initialPose = null;
        Command autoCommand = null;

        switch (selectedAutoName) {
            case "1 Ball":
                initialPose = Trajectories.driveBack30In.getInitialPose();
                autoCommand = new SequentialCommandGroup(
                                new CmdShootAlign().withTimeout(2),
                                trajectoryCmd(Trajectories.driveBack30In));
                break;
            case "2 Ball":
                initialPose = Trajectories.twoBallTraj.getInitialPose();
                autoCommand = new SequentialCommandGroup(
                                    new InstantCommand(() -> intake.ejectIntake(), intake),

                                    new ParallelDeadlineGroup(
                                        trajectoryCmd(Trajectories.twoBallTraj), 
                                        new CmdExtendIntakeAndRun()
                                    ),

                                    new CmdInPlaceTurn(180),

                                    new CmdShootAlign().withTimeout(2));
                break;
            case "3 Ball":
                initialPose = trajectories.get("3Ballv2_i").getInitialPose();
                autoCommand = new SequentialCommandGroup(
                                pathCmd("3Ballv2_i"), 

                                new CmdInPlaceTurn(180),
                                new CmdShootAlign().withTimeout(2),

                                pathCmd("3Ballv2_ii"),

                                new CmdShootTurnVision(-170));
                break;
            case "4 Ball":
                initialPose = trajectories.get("4Ball_Terminal180_i").getInitialPose();
                autoCommand = new SequentialCommandGroup(
                                //drive and intake 1 ball
                                pathCmd("4Ball_Terminal180_i"),  

                                //turn and shoot 2 balls
                                new CmdInPlaceTurn(180),
                                new CmdShoot(),

                                //drive to ball and terminal and intake
                                pathCmd("4Ball_Terminal180_ii"), 

                                new CmdExtendIntakeAndRun().withTimeout(1),

                                //drive to tarmac and shoot
                                trajectoryCmd("Terminal2Tarmac"),
                                new CmdInPlaceTurn(180),
                                new CmdShootAlign().withTimeout(2));
                break;
            case "5 Ball":
                initialPose = trajectories.get("3Ballv2_i").getInitialPose();
                autoCommand = new SequentialCommandGroup(
                                pathCmd("3Ballv2_i"), 

                                new CmdInPlaceTurn(180),
                                new CmdShootAlign().withTimeout(2),

                                pathCmd("3Ballv2_ii"),

                                new CmdShootTurnVision(-170),

                                pathCmd("5Ballv2_i"),
                                
                                new InstantCommand(() -> drive.stop()),

                                trajectoryCmd("5Ballv2_ii"),

                                new CmdShootTurnVision(-170)
                            );
                break;
            case "S2H1":
                initialPose = trajectories.get("S2H2_i").getInitialPose();
                autoCommand = new SequentialCommandGroup(
                                //drive and intake ball

                                pathCmd("S2H2_i"),

                                //turn and shoot
                                new CmdInPlaceTurn(180),
                                new CmdShootAlign().withTimeout(2),

                                //turn and hoard first ball
                                new CmdInPlaceTurn(90),

                                pathCmd("S2H2_ii"),

                                //drive behind hub
                                new CmdInPlaceTurn(-90),
                                trajectoryCmd("S2H1"),

                                //outtake balls behind hub
                                new CmdExtendIntake(),
                                new CmdOuttake(0.5).withTimeout(2));
                break;
            case "S2H2":
                initialPose = trajectories.get("S2H2_i").getInitialPose();
                autoCommand = new SequentialCommandGroup(

                                //drive and intake ball
                                pathCmd("S2H2_i"),

                                //turn and shoot
                                new CmdInPlaceTurn(180),
                                new CmdShootAlign().withTimeout(2),

                                //turn and hoard first ball
                                new CmdInPlaceTurn(90),

                                pathCmd("S2H2_ii"),

                                // turn and hoard second ball
                                new CmdInPlaceTurn(180),

                                pathCmd("S2H2_iii"), 
                                
                                //hide ball behind hub
                                trajectoryCmd("S2H2_iv"),

                                // new CmdInPlaceTurn(130),
                                new CmdExtendIntake(),
                                new CmdOuttake(0.4).withTimeout(1));
                break;
            case "S1H1":
                initialPose = trajectories.get("S1H1_i").getInitialPose();
                autoCommand = new SequentialCommandGroup(          
                                //drive and intake enemy ball          
                                pathCmd("S1H1_i"), 
                                new InstantCommand(() -> drive.stop()),
                                
                                //turn and shoot 1 ball
                                new CmdInPlaceTurn(180),
                                
                                new CmdRetractHopper(),
                                new ParallelCommandGroup(
                                    new CmdAlign(),
                                    new CmdShootSingleBall()
                                ).withTimeout(2),
                                
                                //hide enemy ball behind hub
                                trajectoryCmd("S1H1_ii"),
                                new CmdExtendIntake(),
                                new CmdOuttake(0.5).withTimeout(1));
                break;
            case "S1H2":
                initialPose = trajectories.get("S1H1_i").getInitialPose();
                autoCommand = new SequentialCommandGroup(
                                //drive and intake enemy ball
                                pathCmd("S1H1_i"), 
                                new InstantCommand(() -> drive.stop()),

                                //turn and shoot 1 ball
                                new CmdInPlaceTurn(180),
                                
                                new CmdRetractHopper(),
                                new ParallelCommandGroup(
                                    new CmdAlign(),
                                    new CmdShootSingleBall()
                                ).withTimeout(2),

                                //drive and intake enemy ball
                                pathCmd("S1H2_ii"), 

                                //hide enemy balls behind hub
                                new CmdInPlaceTurn(180),
                    
                                trajectoryCmd("S1H2_iii"), 
                                
                                new CmdExtendIntake(),
                                new CmdOuttake(0.5).withTimeout(2));
                break;
            case "S1I1":
                initialPose = trajectories.get("S1H1_i").getInitialPose();
                autoCommand = new SequentialCommandGroup(
                                //drive and intake enemy ball
                                pathCmd("S1H1_i"), 
                                new InstantCommand(() -> drive.stop()),

                                //turn and shoot 1 ball
                                new CmdInPlaceTurn(180),
                                
                                new CmdRetractHopper(),
                                new ParallelCommandGroup(
                                    new CmdAlign(),
                                    new CmdShootSingleBall()
                                ).withTimeout(2));
                break;
            case "Billiards":
                // initial position: (6.8, 6.272, 45 deg - should be approx. pointing straight at the ball to knock)
                initialPose = new Pose2d(6.8, 6.272, Rotation2d.fromDegrees(45));
                autoCommand = new SequentialCommandGroup (
                                // outtake ball
                                new SequentialCommandGroup(
                                    new CmdExtendIntake(),
                                    new CmdOuttake(0.85).withTimeout(1.5)
                                ).withTimeout(2),

                                //turn, drive, and intake enemy ball
                                new CmdInPlaceTurn(70),

                                pathCmd("Billiards_i"),

                                //turn and eject enemy ball
                                new CmdInPlaceTurn(55),

                                new CmdExtendIntake(),
                                new CmdOuttake(0.6).withTimeout(1.5),

                                //drive and intake ball
                                pathCmd("Billiards_ii"),

                                //turn and shoot
                                new CmdInPlaceTurn(96),

                                new CmdShootAlign().withTimeout(2));
                break;
            default: 
                Log.info("Auto Selector", "Something went wrong in getting the auto name - misspelling?");
                break;
        }

        drive.resetPose(initialPose);
        return autoCommand;
    }
    
    //Drive trajectory and intake balls
    private SequentialCommandGroup pathCmd(String trajectory) {
        ParallelDeadlineGroup movement = new ParallelDeadlineGroup (trajectoryCmd(trajectory), new CmdExtendIntakeAndRun());
        return new SequentialCommandGroup(new InstantCommand(intake::ejectIntake, intake), movement);
    }

    // Helpers to define common commands used in autos
    private Command trajectoryCmd(String trajName) {
        return trajectoryCmd(trajectories.get(trajName));
    }

    private Command trajectoryCmd(Trajectory traj) {
        return new RamseteCommand(
                        traj,
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

    /**
     * pose is the same translation but flipped 180 rotation
     */
    private Pose2d inverseRotation(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), pose.getRotation().unaryMinus());
    }
}
