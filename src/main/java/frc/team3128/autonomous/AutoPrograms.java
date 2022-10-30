package frc.team3128.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.commands.CmdExtendIntake;
import frc.team3128.commands.CmdExtendIntakeAndRun;
import frc.team3128.commands.CmdInPlaceTurn;
import frc.team3128.commands.CmdIntakeCargo;
import frc.team3128.commands.CmdShoot;
import frc.team3128.commands.CmdShootAlign;
import frc.team3128.commands.CmdShootAlignSingle;
import frc.team3128.commands.CmdShootDist;
import frc.team3128.commands.CmdOuttake;
import frc.team3128.commands.CmdShootTurnVision;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;
import frc.team3128.subsystems.Shooter;
import static frc.team3128.autonomous.Trajectories.*;

import java.util.function.Supplier;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    private NAR_Drivetrain drive;
    private Shooter shooter;
    private Intake intake;
    private Hopper hopper;
    private Hood hood;
    private LimelightSubsystem limelights;

    public AutoPrograms() {
        drive = NAR_Drivetrain.getInstance();
        shooter = Shooter.getInstance();
        intake = Intake.getInstance();
        hopper = Hopper.getInstance();
        hood = Hood.getInstance();
        limelights = LimelightSubsystem.getInstance();
        initAutoSelector();
    }

    private void initAutoSelector() {
        String[] autoStrings = new String[] {"Intake 0", "Intake 1", "Intake 2", "4 Ball", "5 Ball", "S2H1", "Intake 1 Hoard 2", "S1H1", "S1I1", "S1H2", "Billiards"};
        NarwhalDashboard.addAutos(autoStrings);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getSelectedAutoName();
        // String selectedAutoName = "2+1 Ball"; // uncomment and change this for testing without opening Narwhal Dashboard

        if (selectedAutoName == null) {
            return null;
        }

        Pose2d initialPose = null;
        Command autoCommand = null;

        switch (selectedAutoName) {
            case "Intake 0":
                initialPose = driveBack30In.getInitialPose();
                autoCommand = new SequentialCommandGroup(
                                new CmdShootAlign().withTimeout(4),
                                trajectoryCmd("driveBack30In"));
                break;
            case "Intake 1":
                initialPose = inverseRotation(get("3Ballv2_i").getInitialPose());
                autoCommand = new SequentialCommandGroup(

                                    new CmdShootAlign().withTimeout(3),

                                    new CmdInPlaceTurn(180),

                                    IntakePathCmd("3Ballv2_i"),
                                    
                                    new CmdShootTurnVision(180));

                                    // IntakePathCmd("twoBallTraj"),

                                    // new CmdInPlaceTurn(180),

                                    // new CmdShootAlign().withTimeout(3));
                break;
            case "Intake 2":
                initialPose = inverseRotation(get("3Ballv2_i").getInitialPose());
                autoCommand = new SequentialCommandGroup(
                                new CmdShootAlign().withTimeout(3),

                                new CmdInPlaceTurn(180),

                                IntakePathCmd("3Ballv2_i"), 

                                new CmdInPlaceTurn(180),

                                IntakePathCmd("3Ballv2_ii"),

                                new CmdShootTurnVision(-170));
                break;
            case "4 Ball":
                initialPose = inverseRotation(get("4Ball_Terminal180_i").getInitialPose());
                autoCommand = new SequentialCommandGroup(
                                new CmdShootAlign().withTimeout(2),
                                new CmdInPlaceTurn(180),
                                //drive and intake 1 ball
                                IntakePathCmd("4Ball_Terminal180_i"),  

                                //turn and shoot 2 balls
                                new CmdInPlaceTurn(180),
                                new CmdShoot(),

                                //drive to ball and terminal and intake
                                IntakePathCmd("4Ball_Terminal180_ii"), 

                                new CmdExtendIntakeAndRun().withTimeout(1), // is this still needed?

                                //drive to tarmac and shoot
                                trajectoryCmd("Terminal2Tarmac"),
                                new CmdInPlaceTurn(180),
                                new CmdShootAlign().withTimeout(2));
                break;
            case "5 Ball":
                initialPose = inverseRotation(get("3Ballv2_i").getInitialPose());
                autoCommand = new SequentialCommandGroup(
                                new CmdShootAlign().withTimeout(2.5),
                                new CmdInPlaceTurn(180),

                                IntakePathCmd("3Ballv2_i"), 

                                new CmdInPlaceTurn(180),
                                new CmdShootAlign().withTimeout(2),

                                IntakePathCmd("3Ballv2_ii"),

                                new CmdShootTurnVision(-170),

                                IntakePathCmd("5Ballv2_i"),
                                
                                new InstantCommand(() -> drive.stop()),

                                trajectoryCmd("5Ballv2_ii"),

                                new CmdShootTurnVision(-170)
                            );
                break;
            case "S2H1":
                initialPose = get("S2H2_i").getInitialPose();
                autoCommand = new SequentialCommandGroup(
                                //drive and intake ball

                                IntakePathCmd("S2H2_i"),

                                //turn and shoot
                                new CmdInPlaceTurn(180),
                                new CmdShootAlign().withTimeout(2),

                                //turn and hoard first ball
                                new CmdInPlaceTurn(90),

                                IntakePathCmd("S2H2_ii"),

                                //drive behind hub
                                new CmdInPlaceTurn(-90),
                                trajectoryCmd("S2H1"),

                                //outtake balls behind hub
                                new CmdExtendIntake(),
                                new CmdOuttake(0.5).withTimeout(2));
                break;
            case "Intake 1 Hoard 2":
                initialPose = inverseRotation(get("S2H2_i").getInitialPose());
                autoCommand = new SequentialCommandGroup(
                                
                                new CmdShootAlign().withTimeout(3),
                                
                                new CmdInPlaceTurn(180),

                                //drive and intake ball
                                IntakePathCmd("S2H2_i"),

                                //turn and shoot
                                new CmdInPlaceTurn(180),

                                new CmdShootAlign().withTimeout(2),

                                //turn and hoard first ball
                                new CmdInPlaceTurn(90),

                                IntakePathCmd("S2H2_ii"),

                                // turn and hoard second ball
                                new CmdInPlaceTurn(180),

                                IntakePathCmd("S2H2_iii"), 
                                
                                //hide ball behind hub
                                trajectoryCmd("S2H2_iv"),

                                // new CmdInPlaceTurn(130),
                                new CmdExtendIntake(),
                                new CmdOuttake(0.4).withTimeout(1));
                break;
            case "S1H1":
                initialPose = get("S1H1_i").getInitialPose();
                autoCommand = new SequentialCommandGroup(          
                                //drive and intake enemy ball          
                                IntakePathCmd("S1H1_i"), 
                                new InstantCommand(() -> drive.stop()),
                                
                                //turn and shoot 1 ball
                                new CmdInPlaceTurn(180),
                                
                                new CmdShootAlignSingle().withTimeout(2),
                                
                                //hide enemy ball behind hub
                                trajectoryCmd("S1H1_ii"),
                                new CmdExtendIntake(),
                                new CmdOuttake(0.5).withTimeout(1));
                break;
            case "S1H2":
                initialPose = inverseRotation(get("S1H1_i").getInitialPose());
                autoCommand = new SequentialCommandGroup(

                                new CmdShootAlign().withTimeout(3),

                                new CmdInPlaceTurn(180),
                                //drive and intake enemy ball
                                IntakePathCmd("S1H1_i"), 
                                new InstantCommand(() -> drive.stop()),

                                //turn and shoot 1 ball
                                new CmdInPlaceTurn(180),

                                //drive and intake enemy ball
                                IntakePathCmd("S1H2_ii"), 

                                //hide enemy balls behind hub
                                new CmdInPlaceTurn(180),
                    
                                trajectoryCmd("S1H2_iii"), 
                                
                                new CmdExtendIntake(),
                                new CmdOuttake(0.5).withTimeout(2));
                break;
            case "S1I1":
                initialPose = get("S1H1_i").getInitialPose();
                autoCommand = new SequentialCommandGroup(
                                //drive and intake enemy ball
                                IntakePathCmd("S1H1_i"), 
                                new InstantCommand(() -> drive.stop()),

                                //turn and shoot 1 ball
                                new CmdInPlaceTurn(180),
                                
                                new CmdShootAlignSingle().withTimeout(2));
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

                                IntakePathCmd("Billiards_i"),

                                //turn and eject enemy ball
                                new CmdInPlaceTurn(55),

                                new CmdExtendIntake(),
                                new CmdOuttake(0.6).withTimeout(1.5),

                                //drive and intake ball
                                IntakePathCmd("Billiards_ii"),

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
    
    /** 
     * Follow trajectory and intake balls along the path
     */
    private SequentialCommandGroup IntakePathCmd(String trajectory) {
        ParallelDeadlineGroup movement = new ParallelDeadlineGroup(
                                            trajectoryCmd(trajectory), 
                                            new CmdExtendIntakeAndRun());
        return new SequentialCommandGroup(new InstantCommand(intake::ejectIntake, intake), new WaitCommand(0.125), movement);
    }

    /**
     * Flip 180 degrees rotation wise but keep same pose translation 
     */
    private Pose2d inverseRotation(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), new Rotation2d(pose.getRotation().getRadians() + Math.PI));
    }
}
