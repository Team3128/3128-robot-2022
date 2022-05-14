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
import frc.team3128.Constants.*;
import static frc.team3128.Constants.HoodConstants.*;
import static frc.team3128.Constants.DriveConstants.*;
import frc.team3128.commands.CmdAlign;
import frc.team3128.commands.CmdExtendIntake;
import frc.team3128.commands.CmdExtendIntakeAndRun;
import frc.team3128.commands.CmdHopperShooting;
import frc.team3128.commands.CmdInPlaceTurn;
import frc.team3128.commands.CmdInPlaceTurnVision;
import frc.team3128.commands.CmdRetractHopper;
import frc.team3128.commands.CmdOuttake;
import frc.team3128.commands.CmdShootDist;
import frc.team3128.commands.CmdShootRPM;
import frc.team3128.commands.CmdShootSingleBall;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Shooter.ShooterState;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.LinkedHashMap;

public class AutoPrograms {

    private class AutoInfo {
        public Command command;
        public Pose2d initialPose;

        public AutoInfo(Command command, Pose2d initialPose) {
            this.command = command;
            this.initialPose = initialPose;
        }
    }

    private NAR_Drivetrain drive;
    private Shooter shooter;
    private Intake intake;
    private Hopper hopper;
    private Hood hood;
    private LimelightSubsystem limelights;

    private HashMap<String, Trajectory> trajectories;
    private HashMap<String, AutoInfo> autoMap;

    private Command auto_1Ball;
    private Command auto_2Ball;
    private Command auto_3Ball180;
    private Command auto_S2H1;
    private Command auto_S2H2;
    private Command auto_4Ball180;
    private Command auto_5Ball180;
    private Command auto_Billiards;
    private Command auto_S1H1;
    private Command auto_S1I1;
    private Command auto_S1H2;

    public AutoPrograms(NAR_Drivetrain drive, Shooter shooter, Intake intake, Hopper hopper, Hood hood, LimelightSubsystem limelights) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        this.hood = hood;
        this.limelights = limelights;

        loadTrajectories();
        initAutos();
        initAutoSelector();
    }

    private void loadTrajectories() {
        trajectories = new HashMap<String, Trajectory>();

        final String[] trajectoryNames = {
            "S2H2_i.wpilib.json",
            "S2H2_ii.wpilib.json",
            "S2H1.wpilib.json",
            "S2H2_iii.wpilib.json",
            "S2H2_iv.wpilib.json",
            "4Ball_Terminal180_i.wpilib.json",
            "4Ball_Terminal180_ii.wpilib.json",
            "Terminal2Tarmac.wpilib.json",
            "Tarmac2Terminal.wpilib.json",
            "Billiards_i.wpilib.json",
            "Billiards_ii.wpilib.json",
            "3Ballv2_i.wpilib.json",
            "3Ballv2_ii.wpilib.json",
            "5Ballv2_i.wpilib.json",
            "5Ballv2_ii.wpilib.json",
            "S1H1_i.wpilib.json",
            "S1H1_ii.wpilib.json",
            "S1H2_ii.wpilib.json",
            "S1H2_iii.wpilib.json"
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

    private void initAutos() {
        autoMap = new LinkedHashMap<String, AutoInfo>();

        auto_1Ball = new SequentialCommandGroup(
                            alignShootCmd(),
                            trajectoryCmd(Trajectories.driveBack30In)
        );

        auto_2Ball = new SequentialCommandGroup(
                            new InstantCommand(() -> intake.ejectIntake(), intake),

                            new ParallelDeadlineGroup(
                                trajectoryCmd(Trajectories.twoBallTraj), 
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            new CmdInPlaceTurn(drive, 180),

                            alignShootCmd()
        );

        auto_3Ball180 = new SequentialCommandGroup(

            new InstantCommand(() -> intake.ejectIntake(), intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("3Ballv2_i.wpilib.json"), 
                new CmdExtendIntakeAndRun(intake, hopper)
            ),

            new CmdInPlaceTurn(drive, 180),
            alignShootCmd(),

            new InstantCommand(() -> intake.ejectIntake(), intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("3Ballv2_ii.wpilib.json"),
                new CmdExtendIntakeAndRun(intake, hopper)
            ),

            turnRightToAligned(),
            alignShootCmd()
        );

        auto_S2H1 = new SequentialCommandGroup(

                            //drive and intake ball
                            new InstantCommand(() -> intake.ejectIntake(), intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_i.wpilib.json"),
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            //turn and shoot
                            new CmdInPlaceTurn(drive, 180),
                            alignShootCmd(),

                            //turn and hoard first ball
                            new CmdInPlaceTurn(drive, 90),
                            new InstantCommand(() -> intake.ejectIntake(), intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_ii.wpilib.json"),
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            //drive behind hub
                            new CmdInPlaceTurn(drive, -90),
                            trajectoryCmd("S2H1.wpilib.json"),

                            //outtake balls behind hub
                            new CmdExtendIntake(intake),
                            new CmdOuttake(intake, hopper, 0.5).withTimeout(2)

        );   

        auto_S2H2 = new SequentialCommandGroup(

                            //drive and intake ball
                            new InstantCommand(() -> intake.ejectIntake(), intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_i.wpilib.json"),
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            //turn and shoot
                            new CmdInPlaceTurn(drive, 180),
                            alignShootCmd(),

                            //turn and hoard first ball
                            new CmdInPlaceTurn(drive, 90),
                            new InstantCommand(() -> intake.ejectIntake(), intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_ii.wpilib.json"),
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            // turn and hoard second ball
                            new CmdInPlaceTurn(drive, 180),
                            new InstantCommand(() -> intake.ejectIntake(), intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_iii.wpilib.json"), 
                                new CmdExtendIntakeAndRun(intake, hopper)),

                            // trajectoryCmd(12),
                            // new ParallelDeadlineGroup(
                            //     trajectoryCmd(13),
                            //     new CmdExtendIntakeAndRun(intake, hopper)
                            // ),
                            
                            //hide ball behinde hub
                            trajectoryCmd("S2H2_iv.wpilib.json"),

                            // new CmdInPlaceTurn(drive, 130),
                            new CmdExtendIntake(intake),
                            new CmdOuttake(intake, hopper, 0.4).withTimeout(1)

        );

        auto_4Ball180 = new SequentialCommandGroup(
                            //drive and intake 1 ball
                            new InstantCommand(() -> intake.ejectIntake(), intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("4Ball_Terminal180_i.wpilib.json"),  
                                new CmdExtendIntakeAndRun(intake, hopper)),

                            //turn and shoot 2 balls
                            new CmdInPlaceTurn(drive, 180),
                            shootCmd(),

                            //drive to ball and terminal and intake
                            new InstantCommand(() -> intake.ejectIntake(), intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("4Ball_Terminal180_ii.wpilib.json"), 
                                new CmdExtendIntakeAndRun(intake, hopper)),
                            new CmdExtendIntakeAndRun(intake, hopper).withTimeout(1),

                            //drive to tarmac and shoot
                            trajectoryCmd("Terminal2Tarmac.wpilib.json"),
                            new CmdInPlaceTurn(drive, 180),
                            alignShootCmd()

        );

        auto_5Ball180 = new SequentialCommandGroup(
            new InstantCommand(() -> intake.ejectIntake(), intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("3Ballv2_i.wpilib.json"), 
                new CmdExtendIntakeAndRun(intake, hopper)
            ),

            new CmdInPlaceTurn(drive, 180),
            alignShootCmd(),

            new InstantCommand(() -> intake.ejectIntake(), intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("3Ballv2_ii.wpilib.json"),
                new CmdExtendIntakeAndRun(intake, hopper)
            ),

            new ParallelCommandGroup(
                turnRightToAligned(),
                shootCmd()
            ),

            new InstantCommand(() -> intake.ejectIntake(), intake),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    trajectoryCmd("5Ballv2_i.wpilib.json"),
                    new InstantCommand(() -> drive.stop())),
                    // new WaitCommand(0.5)),
                new CmdExtendIntakeAndRun(intake, hopper)
            ),

            trajectoryCmd("5Ballv2_ii.wpilib.json"),

            new ParallelCommandGroup(
                turnRightToAligned(),
                shootCmd()
            )
        );

        auto_Billiards = new SequentialCommandGroup (
                            // initial position: (6.8, 6.272, 45 deg - should be approx. pointing straight at the ball to knock)
                            new SequentialCommandGroup(
                                new CmdExtendIntake(intake),
                                new CmdOuttake(intake, hopper, 0.85).withTimeout(1.5)
                            ).withTimeout(2),

                            new CmdInPlaceTurn(drive, 70),
                            
                            new InstantCommand(() -> intake.ejectIntake(), intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("Billiards_i.wpilib.json"),
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            new CmdInPlaceTurn(drive, 55),

                            new CmdExtendIntake(intake),
                            new CmdOuttake(intake, hopper, 0.6).withTimeout(1.5),

                            new InstantCommand(() -> intake.ejectIntake(), intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("Billiards_ii.wpilib.json"),
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            new CmdInPlaceTurn(drive, 96),

                            alignShootCmd()
        );

        auto_S1H1 = new SequentialCommandGroup(

            new InstantCommand(() -> intake.ejectIntake(), intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("S1H1_i.wpilib.json"), 
                new CmdExtendIntakeAndRun(intake, hopper)  
            ),
            new InstantCommand(() -> drive.stop()),

            new CmdInPlaceTurn(drive, 180),
            
            new CmdRetractHopper(hopper).withTimeout(0.5),
            new InstantCommand(() -> shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(limelights::turnShooterLEDOn, limelights),
            new ParallelCommandGroup(
                new CmdAlign(drive, limelights),
                new CmdHopperShooting(hopper, shooter::isReady, 0.6),
                new CmdShootSingleBall(shooter, hood, limelights)
            ).withTimeout(2),
            new InstantCommand(limelights::turnShooterLEDOff, limelights),
            
            trajectoryCmd("S1H1_ii.wpilib.json"),
            new CmdExtendIntake(intake),
            new CmdOuttake(intake, hopper, 0.5).withTimeout(1)
        );

        auto_S1I1 = new SequentialCommandGroup(

            new InstantCommand(() -> intake.ejectIntake(), intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("S1H1_i.wpilib.json"), 
                new CmdExtendIntakeAndRun(intake, hopper)  
            ),
            new InstantCommand(() -> drive.stop()),

            new CmdInPlaceTurn(drive, 180),
            
            new CmdRetractHopper(hopper).withTimeout(0.5),
            new InstantCommand(() -> shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(limelights::turnShooterLEDOn, limelights),
            new ParallelCommandGroup(
                new CmdAlign(drive, limelights),
                new CmdHopperShooting(hopper, shooter::isReady, 0.6),
                new CmdShootSingleBall(shooter, hood, limelights)
            ).withTimeout(2),
            new InstantCommand(limelights::turnShooterLEDOff, limelights)
        );

        auto_S1H2 = new SequentialCommandGroup(
            new InstantCommand(() -> intake.ejectIntake(), intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("S1H1_i.wpilib.json"), 
                new CmdExtendIntakeAndRun(intake, hopper)  
            ),
            new InstantCommand(() -> drive.stop()),

            new CmdInPlaceTurn(drive, 180),
            
            new CmdRetractHopper(hopper).withTimeout(0.5),
            new InstantCommand(() -> shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(limelights::turnShooterLEDOn, limelights),
            new ParallelCommandGroup(
                new CmdAlign(drive, limelights),
                new CmdHopperShooting(hopper, shooter::isReady, 0.6),
                new CmdShootSingleBall(shooter, hood, limelights)
            ).withTimeout(2),
            new InstantCommand(limelights::turnShooterLEDOff, limelights),

            new InstantCommand(() -> intake.ejectIntake(), intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("S1H2_ii.wpilib.json"), 
                new CmdExtendIntakeAndRun(intake, hopper)  
            ),

            new CmdInPlaceTurn(drive, 180),

            trajectoryCmd("S1H2_iii.wpilib.json"), 
            
            new CmdExtendIntake(intake),
            new CmdOuttake(intake, hopper, 0.5).withTimeout(2)
        );
        
        autoMap.put("1 Ball", new AutoInfo(auto_1Ball, Trajectories.driveBack30In.getInitialPose()));
        autoMap.put("2 Ball", new AutoInfo(auto_2Ball, Trajectories.twoBallTraj.getInitialPose()));
        autoMap.put("3 Ball", new AutoInfo(auto_3Ball180, trajectories.get("3Ballv2_i.wpilib.json").getInitialPose()));
        autoMap.put("S2H1", new AutoInfo(auto_S2H1, trajectories.get("S2H2_i.wpilib.json").getInitialPose()));
        autoMap.put("S2H2", new AutoInfo(auto_S2H2, trajectories.get("S2H2_i.wpilib.json").getInitialPose()));
        autoMap.put("4 Ball", new AutoInfo(auto_4Ball180, trajectories.get("4Ball_Terminal180_i.wpilib.json").getInitialPose()));
        autoMap.put("5 Ball", new AutoInfo(auto_5Ball180, trajectories.get("3Ballv2_i.wpilib.json").getInitialPose()));
        autoMap.put("Billiards", new AutoInfo(auto_Billiards, new Pose2d(6.8, 6.272, Rotation2d.fromDegrees(45))));
        autoMap.put("S1H1", new AutoInfo(auto_S1H1, trajectories.get("S1H1_i.wpilib.json").getInitialPose()));
        autoMap.put("S1I1", new AutoInfo(auto_S1I1, trajectories.get("S1H1_i.wpilib.json").getInitialPose()));
        autoMap.put("S1H2", new AutoInfo(auto_S1H2, trajectories.get("S1H1_i.wpilib.json").getInitialPose()));
    }

    private void initAutoSelector() {
        for (String key : autoMap.keySet()) {
            NarwhalDashboard.addAuto(key, autoMap.get(key).command);
        }
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getSelectedAutoName();

        if (selectedAutoName == null) {
            return null;
        }

        AutoInfo selectedInfo = autoMap.get(selectedAutoName);
        drive.resetPose(selectedInfo.initialPose);

        return selectedInfo.command;
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

    private SequentialCommandGroup shootCmd() {
        return new SequentialCommandGroup(
            new CmdRetractHopper(hopper).withTimeout(0.5),
            new InstantCommand(() -> shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(limelights::turnShooterLEDOn, limelights),
            new ParallelCommandGroup(
                new CmdHopperShooting(hopper, shooter::isReady),
                new CmdShootDist(shooter, hood, limelights)
            ).withTimeout(2)
        );
    }
    
    private SequentialCommandGroup shootCmd(int RPM) {
        return new SequentialCommandGroup(
            new CmdRetractHopper(hopper).withTimeout(0.5),
            new InstantCommand(() -> shooter.setState(ShooterState.UPPERHUB)),
            new ParallelCommandGroup(
                new InstantCommand(() -> hood.startPID(HOME_ANGLE)),
                new CmdHopperShooting(hopper, shooter::isReady),
                new CmdShootRPM(shooter, RPM)
            ).withTimeout(2)
        );
    }

    private SequentialCommandGroup shootCmd(int RPM, double angle) {
        return new SequentialCommandGroup(
            new CmdRetractHopper(hopper).withTimeout(0.5),
            new InstantCommand(() -> shooter.setState(ShooterState.UPPERHUB)),
            new ParallelCommandGroup(
                new InstantCommand(() -> hood.startPID(angle)),
                new CmdHopperShooting(hopper, shooter::isReady),
                new CmdShootRPM(shooter, RPM)
            ).withTimeout(2)
        );
    }

    private SequentialCommandGroup alignShootCmd() {
        return new SequentialCommandGroup(
            new CmdRetractHopper(hopper).withTimeout(0.5),
            new InstantCommand(() -> shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(limelights::turnShooterLEDOn, limelights),
            new ParallelCommandGroup(
                new CmdAlign(drive, limelights),
                new CmdHopperShooting(hopper, shooter::isReady),
                new CmdShootDist(shooter, hood, limelights)
            ).withTimeout(2),
            new InstantCommand(limelights::turnShooterLEDOff, limelights)
        );
    }

    private Command turnLeftToAligned() {
        return new SequentialCommandGroup(
            new InstantCommand(limelights::turnShooterLEDOn, limelights),
            new CmdInPlaceTurnVision(drive, limelights, 170),
            new CmdAlign(drive, limelights),
            new InstantCommand(limelights::turnShooterLEDOff, limelights)
        );
    }

    private Command turnRightToAligned() {
        return new SequentialCommandGroup(
            new InstantCommand(limelights::turnShooterLEDOn, limelights),
            new CmdInPlaceTurnVision(drive, limelights, -170),
            new CmdAlign(drive, limelights),
            new InstantCommand(limelights::turnShooterLEDOff, limelights)
        ).withTimeout(3);
    }

    private Pose2d inverseRotation(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), pose.getRotation().unaryMinus());
    }
}
