package frc.team3128;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.team3128.Constants.*;
import frc.team3128.autonomous.Trajectories;
import frc.team3128.commands.*;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.*;
import frc.team3128.subsystems.Shooter.ShooterState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private NAR_Drivetrain m_drive;
    private Shooter m_shooter;
    private Intake m_intake;   
    private Hopper m_hopper;
    private Climber m_climber;

    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

    private Limelight m_shooterLimelight;
    private Limelight m_ballLimelight;

    // private String[] trajJson = Filesystem.getDeployDirectory().toPath().resolve("paths").toFile().list();
    // private String[] trajJson = (new File(Filesystem.getLaunchDirectory(), "src" + File.separator + "main" + File.separator + "deploy")).list();
    private String[] trajJson = {
        "2_BallBot_i.wpilib.json", // 0
        "2_BallMid_i.wpilib.json",
        "2_BallTop_i.wpilib.json", // 2
        "3_Ball_i.wpilib.json",
        "3_Ball_ii.wpilib.json", // 4
        "3_BallHK_i.wpilib.json",
        "3_BallHK_ii.wpilib.json", // 6
        "3_BallTerm_i.wpilib.json",
        "3_BallTerm_ii.wpilib.json", // 8
        "4_Ball_i.wpilib.json",
        "4_Ball_ii.wpilib.json", // 10
        "4_BallE_i.wpilib.json",
        "4_BallE_ii.wpilib.json", // 12
        "4_BallTerm_i.wpilib.json",
        "4_BallTerm_ii.wpilib.json", // 14
        "4_BallTerm_iii.wpilib.json",
        "4_BallTerm_iv.wpilib.json", // 16
        "5_Ball_i.wpilib.json",
        "5_Ball_ii.wpilib.json", // 18
        "5_Ball_iii.wpilib.json",
        "5_Ball_iv.wpilib.json", // 20
        "leaveTerm_i.wpilib.json",
        "leaveTerm_ii.wpilib.json"
    };
    private Trajectory[] trajectory = new Trajectory[trajJson.length];
    
    private SequentialCommandGroup extendIntakeAndReverse;
    private Command shootCommand;
    private SequentialCommandGroup manualShoot;
    private SequentialCommandGroup lowerHubShoot;
    private CmdClimb climbCommand;
    private CmdClimbTraversal climbTraversalCommand;

    private HashMap<Command, Pose2d> initialPoses;

    private Command auto_1Ball, auto_2BallTop, auto_2BallMid, auto_2BallBot, auto_3BallTerminal, auto_3BallHook, auto_3BallHersheyKiss, auto_4BallE, auto_4BallTerm, auto_5Ball;

    private boolean DEBUG = true;
    private boolean driveHalfSpeed = false;

    public RobotContainer() {
        
        m_drive = NAR_Drivetrain.getInstance();
        m_shooter = Shooter.getInstance();
        m_intake = Intake.getInstance();
        m_hopper = Hopper.getInstance();
        m_climber = Climber.getInstance();

        //Enable all PIDSubsystems so that useOutput runs
        m_shooter.enable();

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);

        m_shooterLimelight = new Limelight("limelight-cog", VisionConstants.TOP_CAMERA_ANGLE, 
                                                             VisionConstants.TOP_CAMERA_HEIGHT, 
                                                            VisionConstants.TOP_FRONT_DIST, 0); 
        m_ballLimelight = new Limelight("limelight-sog", VisionConstants.BALL_LL_ANGLE, 
                                                        VisionConstants.BALL_LL_HEIGHT, 
                                                        VisionConstants.BALL_LL_FRONT_DIST, 0);

        m_commandScheduler.setDefaultCommand(m_drive, new CmdArcadeDrive(m_drive, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle, () -> driveHalfSpeed));
        //m_commandScheduler.setDefaultCommand(m_hopper, new CmdHopperDefault(m_hopper, m_shooter::isReady)); //TODO: make input into this good method ???


        initAutos();
        initDashboard();
        initLimelights(m_shooterLimelight, m_ballLimelight); 
        configureButtonBindings();
        
        if(RobotBase.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        // Buttons...
        // right:
        // 1 (trigger): shoot upper hub
        // 2: intake 
        // 3: ball pursuit
        // 4: shoot lower hub
        // 5: climb from mid to high
        // 6: extend climber elev to height for mid to high climb
        // 7: retract climber elev to 0
        // 8: reverse intake
        //
        // left:
        // 2: reset climber encoder
        //
        // 5: extend climber slightly (for hooking stationary hooks while climbing)
        // 8: extend climber to diagonal extension
        // 9: extend climber to top
        // 10: retract climber to 0
        // 11: engage friction break
        // 12: extend climber piston
        // 13: climber go up while held
        // 14: climber go down while held 
        // 15: retract climber piston
        // 16: disengage friction break

        //RIGHT
        m_rightStick.getButton(1).whenPressed(shootCommand)
                                .whenReleased(new ParallelCommandGroup(new InstantCommand(m_shooter::stopShoot, m_shooter), new InstantCommand(m_shooterLimelight::turnLEDOff)));

        m_rightStick.getButton(2).whenHeld(new CmdExtendIntakeAndRun(m_intake, m_hopper));
        
        // m_rightStick.getButton(3).whenHeld(new ParallelCommandGroup(
        //                                     new CmdBallJoystickPursuit(m_drive, m_ballLimelight, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle),
        //                                     new CmdExtendIntakeAndRun(m_intake, m_hopper)).beforeStarting(new WaitCommand(0.5)) // Wait 0.5s, then extend intake so as to not block vision
        //                                 );

        m_rightStick.getButton(4).whenHeld(lowerHubShoot);

        //m_rightStick.getButton(5).whenPressed(climbCommand);
        m_rightStick.getButton(5).whenPressed(climbTraversalCommand);

        m_rightStick.getButton(6).whenPressed(new CmdClimbEncoder(m_climber, ClimberConstants.CLIMB_ENC_TO_TOP));

        m_rightStick.getButton(7).whenPressed(new CmdClimbEncoder(m_climber, 0));

        m_rightStick.getButton(8).whenHeld(extendIntakeAndReverse);

        m_rightStick.getButton(10).whenPressed(new InstantCommand(m_climber::bothStop, m_climber));

        m_rightStick.getButton(13).whenHeld(new SequentialCommandGroup(
            new CmdRetractHopper(m_hopper),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new ParallelCommandGroup(
                new RunCommand(m_drive::stop, m_drive),
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootRPM(m_shooter, 3000))));

        m_rightStick.getButton(12).whenHeld(new SequentialCommandGroup(
            new CmdRetractHopper(m_hopper),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new ParallelCommandGroup(
                new RunCommand(m_drive::stop, m_drive),
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootRPM(m_shooter, 4000))));
               
        m_rightStick.getButton(14).whenHeld(new SequentialCommandGroup(
            new CmdRetractHopper(m_hopper),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new ParallelCommandGroup(
                new RunCommand(m_drive::stop, m_drive),
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootRPM(m_shooter, 5000))));

        m_rightStick.getButton(15).whenHeld(new SequentialCommandGroup(
            new CmdRetractHopper(m_hopper),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new ParallelCommandGroup(
                new RunCommand(m_drive::stop, m_drive),
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootRPM(m_shooter, 5500))));

        m_rightStick.getButton(11).whenPressed(new CmdExtendIntake(m_intake));
        m_rightStick.getButton(16).whenPressed(() -> m_intake.retractIntake());
 
        //LEFT

        m_leftStick.getButton(1).whenHeld(new SequentialCommandGroup(
            new CmdRetractHopper(m_hopper),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            // new CmdExtendIntake(m_intake),
            new ParallelCommandGroup(
                // new CmdAlign(m_drive, m_shooterLimelight),
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootRPM(m_shooter, 4400))))
        .whenReleased(() -> m_shooterLimelight.turnLEDOff());

        m_leftStick.getButton(2).whenPressed(new InstantCommand(m_climber::resetLeftEncoder, m_climber));        

        m_leftStick.getButton(3).whenPressed(() -> driveHalfSpeed = !driveHalfSpeed);

        // m_leftStick.getButton(5).whenPressed(new CmdClimbEncoder(m_climber, -m_climber.getDesiredTicks(ClimberConstants.SMALL_VERTICAL_DISTANCE)));

        m_leftStick.getButton(11).whenPressed(new InstantCommand(m_climber::bothManualExtend, m_climber))
                                .whenReleased(new InstantCommand(m_climber::bothStop, m_climber));

        m_leftStick.getButton(16).whenPressed(new InstantCommand(m_climber::bothManualRetract, m_climber))
                                .whenReleased(new InstantCommand(m_climber::bothStop, m_climber));

        m_leftStick.getButton(13).whenPressed(new InstantCommand(m_climber::bothExtend, m_climber))
                                .whenReleased(new InstantCommand(m_climber::bothStop, m_climber));

        m_leftStick.getButton(14).whenPressed(new InstantCommand(m_climber::bothRetract, m_climber))
                                .whenReleased(new InstantCommand(m_climber::bothStop, m_climber));

        m_leftStick.getButton(12).whenPressed(new InstantCommand(m_climber::extendPiston, m_climber));
        m_leftStick.getButton(15).whenPressed(new InstantCommand(m_climber::retractPiston, m_climber));
        // m_leftStick.getButton(11).whenPressed(new InstantCommand(m_climber::engageBreak, m_climber));
        // m_leftStick.getButton(16).whenPressed(new InstantCommand(m_climber::disengageBreak, m_climber));

        m_leftStick.getButton(9).whenPressed(new CmdClimbEncoder(m_climber, ClimberConstants.CLIMB_ENC_DIAG_EXTENSION));
        m_leftStick.getButton(8).whenPressed(new CmdClimbEncoder(m_climber, ClimberConstants.CLIMB_ENC_TO_TOP));
        m_leftStick.getButton(10).whenPressed(new CmdClimbEncoder(m_climber, -120));



    }

    public void init() {
        initPneumatics();
        m_shooterLimelight.turnLEDOff();
    }

    private void initAutos() {

        // Arrays.sort(trajJson);

        try {
            for (int i = 0; i < trajJson.length; i++) {
                // Get a path from the string specified in trajJson, and load it into trajectory[i]
                Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajJson[i]);
                trajectory[i] = TrajectoryUtil.fromPathweaverJson(path);
                Log.info("InitAutos", "Trajectory" + i + " = path" + path.toString());
            }
        } catch (IOException ex) {
            DriverStation.reportError("IOException opening trajectory:", ex.getStackTrace());
        }

        initialPoses = new HashMap<Command, Pose2d>();

        climbCommand = new CmdClimb(m_climber);
        climbTraversalCommand = new CmdClimbTraversal(m_climber);
        
        extendIntakeAndReverse = new SequentialCommandGroup(new CmdExtendIntake(m_intake).withTimeout(0.1), new CmdReverseIntake(m_intake, m_hopper));


        //this shoot command is the ideal one with all capabilities
        shootCommand = new SequentialCommandGroup(
                        new InstantCommand(m_shooterLimelight::turnLEDOn),
                        new CmdRetractHopper(m_hopper), 
                        new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
                        // new CmdExtendIntake(m_intake),
                        new ParallelCommandGroup(
                            // new RunCommand(m_intake::runIntake, m_intake),
                            new CmdAlign(m_drive, m_shooterLimelight), 
                            new CmdHopperShooting(m_hopper, m_shooter::isReady),
                            new CmdShootDist(m_shooter, m_shooterLimelight)
                        )
        );

        //use this shoot command for testing
        manualShoot = new SequentialCommandGroup(
                        new InstantCommand(m_shooterLimelight::turnLEDOn), 
                        new CmdRetractHopper(m_hopper),
                        new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
                        new ParallelCommandGroup(
                            new CmdHopperShooting(m_hopper, m_shooter::isReady),
                            new CmdShootDist(m_shooter, m_shooterLimelight)
                        )
        );

        lowerHubShoot = new SequentialCommandGroup(
                            new CmdRetractHopper(m_hopper),
                            new InstantCommand(() -> m_shooter.setState(ShooterState.LOWERHUB)),
                            // new CmdExtendIntake(m_intake),
                            new ParallelCommandGroup(
                                // new RunCommand(m_intake::runIntake, m_intake),
                                new RunCommand(m_drive::stop, m_drive),
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter, 1100))
        );


        //AUTONOMOUS ROUTINES

        auto_1Ball = new SequentialCommandGroup(
                            retractHopperAndShootCmdLL(3750),

                            new RamseteCommand(Trajectories.driveBack30In, 
                            m_drive::getPose,
                            new RamseteController(Constants.DriveConstants.RAMSETE_B, Constants.DriveConstants.RAMSETE_ZETA),
                            new SimpleMotorFeedforward(Constants.DriveConstants.kS,
                                                        Constants.DriveConstants.kV,
                                                        Constants.DriveConstants.kA),
                            Constants.DriveConstants.DRIVE_KINEMATICS,
                            m_drive::getWheelSpeeds,
                            new PIDController(Constants.DriveConstants.RAMSETE_KP, 0, 0),
                            new PIDController(Constants.DriveConstants.RAMSETE_KP, 0, 0),
                            m_drive::tankDriveVolts,
                            m_drive)
        );

        auto_2BallBot = new SequentialCommandGroup(

                            //pick up 1 ball
                            new ParallelDeadlineGroup(
                                trajectoryCmd(0).andThen(m_drive::stop, m_drive),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            //shoot preloaded + first
                            retractHopperAndShootCmd(3250)

        );
        
        auto_2BallMid = new SequentialCommandGroup(

                             //pick up 1 ball
                            new ParallelDeadlineGroup(
                                trajectoryCmd(1).andThen(m_drive::stop, m_drive),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            //shoot first + preloaded
                            retractHopperAndShootCmd(3250)

        );

        auto_2BallTop = new SequentialCommandGroup(

                            //pick up 1 ball

                            new CmdExtendIntake(m_intake),

                            new ParallelDeadlineGroup(
                                trajectoryCmd(2).andThen(m_drive::stop, m_drive),
                                new CmdIntakeCargo(m_intake, m_hopper)
                            ),

                            new InstantCommand(() -> m_intake.retractIntake()),

                            //shoot first + preloaded
                            retractHopperAndShootCmdLL(3750)

        );

        auto_3BallHook = new SequentialCommandGroup(

                            //shoot preloaded ball
                            retractHopperAndShootCmd(3350),

                            //pick up two balls
                            new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                    trajectoryCmd(3),
                                    trajectoryCmd(4),
                                    new InstantCommand(m_drive::stop, m_drive)
                                ),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            //shoot two balls
                            retractHopperAndShootCmd(3250)

        );

        auto_3BallHersheyKiss = new SequentialCommandGroup(
            
                            //shoot preload
                            retractHopperAndShootCmd(3500),
                            
                            //pick up two balls
                            new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                    trajectoryCmd(5),
                                    trajectoryCmd(6),
                                    new InstantCommand(m_drive::stop, m_drive)
                                ),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            //shoot two balls
                            retractHopperAndShootCmdLL(3500)
        );
        
        auto_3BallTerminal = new SequentialCommandGroup(

                            //shoot preloaded ball
                            retractHopperAndShootCmd(3000),

                            trajectoryCmd(7),
                            new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                    trajectoryCmd(8),
                                    new InstantCommand(m_drive::stop, m_drive),
                                    new WaitCommand(1)
                                ),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),
                            
                            trajectoryCmd(19),
                            trajectoryCmd(20),
                            new InstantCommand(m_drive::stop, m_drive),

                            //shoot two balls
                            retractHopperAndShootCmd(3000)

        );

        auto_4BallE = new SequentialCommandGroup(

                            //pick up first ball
                            new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                    trajectoryCmd(11),
                                    new InstantCommand(m_drive::stop, m_drive)
                                ),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            //shoot first + preloaded
                            retractHopperAndShootCmd(3000),

                            //pick up two more balls
                            new CmdExtendIntake(m_intake).withTimeout(0.1),
                            new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                    trajectoryCmd(12),
                                    new InstantCommand(m_drive::stop, m_drive)
                                ),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            //shoot two more balls
                            retractHopperAndShootCmd(3250)

        );

        auto_4BallTerm = new SequentialCommandGroup(
                            //pick up first ball

                            new CmdExtendIntake(m_intake),

                            new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                    trajectoryCmd(13),
                                    new InstantCommand(m_drive::stop, m_drive)
                                ),
                                new CmdIntakeCargo(m_intake, m_hopper)
                            ),

                            new CmdRetractHopper(m_hopper),

                            retractHopperAndShootCmd(3750),
                            new CmdExtendIntake(m_intake),

                            new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                    trajectoryCmd(14),
                                    new InstantCommand(m_drive::stop, m_drive),
                                    new WaitCommand(0.5)
                                ),
                                new CmdIntakeCargo(m_intake, m_hopper)
                            ),

                            new CmdRetractHopper(m_hopper),

                            trajectoryCmd(15),
                            trajectoryCmd(16),
                            new InstantCommand(m_drive::stop, m_drive),

                            //shoot two balls
                            retractHopperAndShootCmd(3750)
        );

        auto_5Ball = new SequentialCommandGroup(

                            new SequentialCommandGroup(
                                new CmdRetractHopper(m_hopper).withTimeout(0.5),
                                new ParallelCommandGroup(
                                    new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                    new CmdShootRPM(m_shooter, 3250)
                                ).withTimeout(1)
                            ),

                            //pick up first ball
                            new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                    trajectoryCmd(15),
                                    trajectoryCmd(16),
                                    new InstantCommand(m_drive::stop, m_drive)
                                ),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            retractHopperAndShootCmd(3750),
                            
                            trajectoryCmd(17),

                            new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                    trajectoryCmd(18),
                                    new InstantCommand(m_drive::stop, m_drive),
                                    new WaitCommand(0.5)
                                ),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            trajectoryCmd(19),
                            trajectoryCmd(20),
                            new InstantCommand(m_drive::stop, m_drive),

                            //shoot two balls
                            retractHopperAndShootCmd(3750)
        );

        // Setup auto-selector
        NarwhalDashboard.addAuto("1 Ball", auto_1Ball);
        NarwhalDashboard.addAuto("2 Ball Bottom", auto_2BallBot);
        NarwhalDashboard.addAuto("2 Ball Mid", auto_2BallMid);
        NarwhalDashboard.addAuto("2 Ball Top", auto_2BallTop);
        NarwhalDashboard.addAuto("3 Ball Hook", auto_3BallHook);
        NarwhalDashboard.addAuto("3 Ball Terminal", auto_3BallTerminal);
        NarwhalDashboard.addAuto("3 Ball Hershey Kiss", auto_3BallHersheyKiss);
        NarwhalDashboard.addAuto("4 Ball E", auto_4BallE);
        NarwhalDashboard.addAuto("4 Ball Terminal", auto_4BallTerm);
        NarwhalDashboard.addAuto("5 Ball", auto_5Ball);
    }

    // Helper for initAutos so we don't clog it up with all of these params
    private RamseteCommand trajectoryCmd(int i) {
        return new RamseteCommand(trajectory[i], 
                            m_drive::getPose,
                            new RamseteController(Constants.DriveConstants.RAMSETE_B, Constants.DriveConstants.RAMSETE_ZETA),
                            new SimpleMotorFeedforward(Constants.DriveConstants.kS,
                                                        Constants.DriveConstants.kV,
                                                        Constants.DriveConstants.kA),
                            Constants.DriveConstants.DRIVE_KINEMATICS,
                            m_drive::getWheelSpeeds,
                            new PIDController(Constants.DriveConstants.RAMSETE_KP, 0, 0),
                            new PIDController(Constants.DriveConstants.RAMSETE_KP, 0, 0),
                            m_drive::tankDriveVolts,
                            m_drive);
    }

    private SequentialCommandGroup retractHopperAndShootCmd(int RPM) {
        return new SequentialCommandGroup(
            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            // new CmdExtendIntake(m_intake),
            new ParallelCommandGroup(
                // new RunCommand(m_intake::runIntake, m_intake),
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootRPM(m_shooter, RPM)
            ).withTimeout(3)
        );
    }

    private SequentialCommandGroup retractHopperAndShootCmdLL(int RPM) {
        return new SequentialCommandGroup(
            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(m_shooterLimelight::turnLEDOn),
            // new CmdExtendIntake(m_intake),
            new ParallelCommandGroup(
                // new RunCommand(m_intake::runIntake, m_intake),
                new CmdAlign(m_drive, m_shooterLimelight),
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootRPM(m_shooter, RPM)
            ).withTimeout(3),
            new InstantCommand(m_shooterLimelight::turnLEDOff)
        );
    }

    private void initDashboard() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            SmartDashboard.putData("Drivetrain", m_drive);
            SmartDashboard.putData("Intake", m_intake);
            SmartDashboard.putData("Hopper", m_hopper);
            SmartDashboard.putData("Shooter", (PIDSubsystem)m_shooter);
        }

        NarwhalDashboard.setSelectedLimelight(m_ballLimelight);
        NarwhalDashboard.startServer();       
    }

    public void stopDrivetrain() {
        m_drive.stop();
    }
  
    private void initLimelights(Limelight... limelightList) {
        Log.info("NarwhalRobot", "Setting up limelight chooser...");
      
        for (Limelight ll : limelightList) {
            NarwhalDashboard.addLimelight(ll);
            ll.turnLEDOff();
        }
    }

    public void updateDashboard() {
        NarwhalDashboard.put("time", Timer.getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("rpm", m_shooter.getMeasurement());
        NarwhalDashboard.put("range", m_shooterLimelight.calculateDistToTopTarget(Constants.VisionConstants.TARGET_HEIGHT));
        SmartDashboard.putNumber("range", m_shooterLimelight.calculateDistToTopTarget(Constants.VisionConstants.TARGET_HEIGHT));
        SmartDashboard.putNumber("ty", m_shooterLimelight.getValue(LimelightKey.VERTICAL_OFFSET, 2));
        SmartDashboard.putNumber("adjusted ty", m_shooterLimelight.getValue(LimelightKey.VERTICAL_OFFSET, 5) * (2/3));

        SmartDashboard.putBoolean("Shooter is ready", m_shooter.isReady());
        SmartDashboard.putString("Shooter state", m_shooter.getState().toString());
        SmartDashboard.putNumber("Shooter Setpoint", m_shooter.getSetpoint());
        SmartDashboard.putNumber("Shooter RPM", m_shooter.getMeasurement());

        SmartDashboard.putString("Intake state:", m_intake.getSolenoid());

        SmartDashboard.putString("Drive half speed", String.valueOf(driveHalfSpeed));
    }

    public Command getAutonomousCommand() {
        
        // I don't understand why putting this in the constructor or initAutos doesn't work, so I put it here and it works (in sim)
        initialPoses.put(auto_1Ball, Trajectories.driveBack30In.getInitialPose());
        initialPoses.put(auto_2BallBot, trajectory[0].getInitialPose());
        initialPoses.put(auto_2BallMid, trajectory[1].getInitialPose());
        initialPoses.put(auto_2BallTop, trajectory[2].getInitialPose());
        initialPoses.put(auto_3BallHook, trajectory[3].getInitialPose());
        initialPoses.put(auto_3BallHersheyKiss, trajectory[5].getInitialPose());
        initialPoses.put(auto_3BallTerminal, trajectory[7].getInitialPose());
        initialPoses.put(auto_4BallE, trajectory[11].getInitialPose());
        initialPoses.put(auto_4BallTerm, trajectory[13].getInitialPose());
        initialPoses.put(auto_5Ball, trajectory[15].getInitialPose());

        Command dashboardSelectedAuto = NarwhalDashboard.getSelectedAuto();

        if (dashboardSelectedAuto == null) {
            return null;
        }

        m_drive.resetPose(initialPoses.get(dashboardSelectedAuto));
        return dashboardSelectedAuto;

        // m_drive.resetPose(trajectory[5].getInitialPose());
        // return auto_3BallHersheyKiss;

    }

    public void initPneumatics() {
        m_climber.retractPiston();
        m_climber.disengageBreak();
        m_intake.retractIntake();
    }

}