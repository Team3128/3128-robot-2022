package frc.team3128;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.Constants.HoodConstants;
import frc.team3128.ConstantsInt.ClimberConstants;
import frc.team3128.ConstantsInt.VisionConstants;
import frc.team3128.autonomous.Trajectories;
import frc.team3128.commands.CmdAlign;
import frc.team3128.commands.CmdArcadeDrive;
import frc.team3128.commands.CmdClimb;
import frc.team3128.commands.CmdClimbEncoder;
import frc.team3128.commands.CmdClimbTraversalOG;
import frc.team3128.commands.CmdExtendIntake;
import frc.team3128.commands.CmdExtendIntakeAndRun;
import frc.team3128.commands.CmdHopperShooting;
import frc.team3128.commands.CmdInPlaceTurn;
import frc.team3128.commands.CmdIntakeCargo;
import frc.team3128.commands.CmdOuttake;
import frc.team3128.commands.CmdRetractHopper;
import frc.team3128.commands.CmdShootDist;
import frc.team3128.commands.CmdShootRPM;
import frc.team3128.commands.CmdShootSingleBall;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.NAR_Drivetrain;
import frc.team3128.subsystems.Shooter;
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
    private Hood m_hood;

    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

    private Limelight m_shooterLimelight;
    private Limelight m_ballLimelight;


    private String[] pathsToLoad = {
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

    private HashMap<String, Trajectory> trajectoryMap = new HashMap<String, Trajectory>();
    
    private SequentialCommandGroup extendIntakeAndReverse;
    private Command shootCommand;

    private SequentialCommandGroup manualShoot;
    private SequentialCommandGroup lowerHubShoot;
    private CmdClimb climbCommand;
    private CmdClimbTraversalOG climbTraversalCommand;

    private HashMap<Command, Pose2d> initialPoses;

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

    private boolean DEBUG = true;
    private boolean driveHalfSpeed = false;

    public RobotContainer() {
        ConstantsInt.initTempConstants();
        m_drive = NAR_Drivetrain.getInstance();
        m_shooter = Shooter.getInstance();
        m_intake = Intake.getInstance();
        m_hopper = Hopper.getInstance();
        m_climber = Climber.getInstance();
        m_hood = Hood.getInstance();

        //Enable all PIDSubsystems so that useOutput runs
        m_shooter.enable();
        m_hood.enable();

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
        // as of 3/27/22
        //
        // right:
        // POV 0: reset drive odometry
        // 1 (trigger): shoot upper hub
        // 2: intake 
        // 3: shoot lower hub
        // 4: ram shot at 2350 rpm ??? 
        // 5: automatically set-up traversal climb
        // 6: extend climber elev to height for mid to high climb
        // 7: retract climber elev to 0
        // 8: reverse intake
        // 10: stop climber
        // 11: increase shooter rpm offest
        // 12: shoot at 4k rpm
        // 13: pid shooter hood to bottom
        // 14: shoot at 5k rpm
        // 15: shoot at 5.5k rpm
        // 16: decrease shooter rpm offset
        //
        // left:
        // POV 0: turn on shooter limelight
        // POV 4: turn off shooter limelight
        // 1: ram shot at 2700 rpm ???
        // 2: reset climber encoder
        // 3: toggle slow driving
        // 5: zero shooter hood encoder
        // 8: extend climber to diagonal extension
        // 9: extend climber to top
        // 10: retract climber to bottom ???
        // 11: climber go up slowly while held
        // 12: extend climber piston
        // 13: climber go up while held
        // 14: climber go down while held 
        // 15: retract climber piston
        // 16: climber go down slowly while held

        //RIGHT
        m_rightStick.getButton(1).whenPressed(shootCommand)
                                .whenReleased(new ParallelCommandGroup(
                                    new InstantCommand(m_shooter::stopShoot, m_shooter),
                                    new InstantCommand(m_shooterLimelight::turnLEDOff)));
        // m_rightStick.getButton(1).whenPressed(new SequentialCommandGroup(new CmdRetractHopper(m_hopper).withTimeout(0.5), new ParallelCommandGroup(new InstantCommand(() -> m_hood.startPID(12)), new CmdShootRPM(m_shooter, 2700), new CmdHopperShooting(m_hopper, m_shooter::isReady))))
        //                             .whenReleased(new ParallelCommandGroup(new InstantCommand(m_shooter::stopShoot, m_shooter)));

        m_rightStick.getButton(2).whenHeld(new CmdExtendIntakeAndRun(m_intake, m_hopper))
                                .whenReleased(new CmdIntakeCargo(m_intake, m_hopper).withTimeout(0.25));
        
        // m_rightStick.getButton(3).whenHeld(new ParallelCommandGroup(
        //                                     new CmdBallJoystickPursuit(m_drive, m_ballLimelight, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle),
        //                                     new CmdExtendIntakeAndRun(m_intake, m_hopper)).beforeStarting(new WaitCommand(0.5)) // Wait 0.5s, then extend intake so as to not block vision
        //                                 );

        m_rightStick.getButton(3).whenHeld(lowerHubShoot);

        m_rightStick.getButton(4).whenPressed(new SequentialCommandGroup(
                                                new CmdRetractHopper(m_hopper).withTimeout(0.5), 
                                                new ParallelCommandGroup(
                                                        new InstantCommand(() -> m_hood.startPID(7)),
                                                        new CmdShootRPM(m_shooter, 2800), 
                                                        new CmdHopperShooting(m_hopper, m_shooter::isReady))))
                                    .whenReleased(new ParallelCommandGroup(new InstantCommand(m_shooter::stopShoot, m_shooter)));

        //m_rightStick.getButton(4).whenHeld(lowerHubShoot);

        //m_rightStick.getButton(5).whenPressed(climbCommand);
        m_rightStick.getButton(5).whenPressed(climbTraversalCommand);

        m_rightStick.getButton(6).whenPressed(new CmdClimbEncoder(m_climber, ClimberConstants.CLIMB_ENC_TO_TOP));

        m_rightStick.getButton(7).whenPressed(new CmdClimbEncoder(m_climber, 0));

        m_rightStick.getButton(8).whenHeld(extendIntakeAndReverse);

        m_rightStick.getButton(10).whenPressed(new InstantCommand(m_climber::bothStop, m_climber));

        m_rightStick.getButton(13).whenPressed(() -> m_hood.startPID(HoodConstants.MIN_ANGLE));
        m_rightStick.getButton(14).whenPressed(() -> m_hood.startPID(HoodConstants.MAX_ANGLE));


        // m_rightStick.getButton(13).whenHeld(new SequentialCommandGroup(
        //     new CmdRetractHopper(m_hopper),
        //     new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
        //     new ParallelCommandGroup(
        //         new RunCommand(m_drive::stop, m_drive),
        //         new CmdHopperShooting(m_hopper, m_shooter::isReady),
        //         new CmdShootRPM(m_shooter, 3000))));

        m_rightStick.getButton(12).whenHeld(new SequentialCommandGroup(
            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new ParallelCommandGroup(
                new RunCommand(m_drive::stop, m_drive),
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootRPM(m_shooter, 4000))));
               
        // m_rightStick.getButton(14).whenHeld(new SequentialCommandGroup(
        //     new CmdRetractHopper(m_hopper).withTimeout(0.5),
        //     new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
        //     new ParallelCommandGroup(
        //         new RunCommand(m_drive::stop, m_drive),
        //         new CmdHopperShooting(m_hopper, m_shooter::isReady),
        //         new CmdShootRPM(m_shooter, 5000))));

        m_rightStick.getButton(15).whenHeld(new SequentialCommandGroup(
            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new ParallelCommandGroup(
                new RunCommand(m_drive::stop, m_drive),
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootRPM(m_shooter, 5500))));

        // m_rightStick.getButton(11).whenPressed(new CmdExtendIntake(m_intake));
        // m_rightStick.getButton(16).whenPressed(() -> m_intake.retractIntake());

        m_rightStick.getButton(11).whenPressed(() -> m_shooter.ratio += 50);
        m_rightStick.getButton(16).whenPressed(() -> m_shooter.ratio -= 50);
 
        //LEFT

        m_leftStick.getButton(1).whenPressed(new SequentialCommandGroup(
                            new CmdRetractHopper(m_hopper).withTimeout(0.5), 
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> m_hood.startPID(ConstantsInt.ShooterConstants.SET_ANGLE)),
                                    new CmdShootRPM(m_shooter, 2800), 
                                    new CmdHopperShooting(m_hopper, m_shooter::isReady))))
                .whenReleased(new ParallelCommandGroup(new InstantCommand(m_shooter::stopShoot, m_shooter)));

        // m_leftStick.getButton(1).whenPressed(new SequentialCommandGroup(new CmdRetractHopper(m_hopper), new ParallelCommandGroup(new InstantCommand(() -> m_hood.startPID(3)), new CmdShootRPM(m_shooter, 2800), new CmdHopperShooting(m_hopper, m_shooter::isReady))))
        //                             .whenReleased(new ParallelCommandGroup(new InstantCommand(m_shooter::stopShoot, m_shooter)));

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

        // should probably remove this
        m_leftStick.getPOVButton(0).whenPressed(() -> m_drive.resetPose());

        // m_leftStick.getButton(4).whenPressed(() -> m_hood.startPID(31));
        m_leftStick.getButton(5).whenPressed(() -> m_hood.zeroEncoder());

        m_rightStick.getPOVButton(0).whenPressed(() -> m_shooterLimelight.turnLEDOn());
        m_rightStick.getPOVButton(4).whenPressed(() -> m_shooterLimelight.turnLEDOff());
    }

    public void init() {
        initPneumatics();
        m_hood.zero();
        // m_shooterLimelight.turnLEDOff();
    }

    private void initAutos() {

        try {
            for (String pathName : pathsToLoad) {
                // Get a path from the string specified in trajJson, and load it into trajectory[i]
                Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(pathName);
                trajectoryMap.put(pathName, TrajectoryUtil.fromPathweaverJson(path));
            }
        } catch (IOException ex) {
            DriverStation.reportError("IOException opening trajectory:", ex.getStackTrace());
        }
        initialPoses = new HashMap<Command, Pose2d>();

        climbCommand = new CmdClimb(m_climber);
        climbTraversalCommand = new CmdClimbTraversalOG(m_climber);
        
        extendIntakeAndReverse = new SequentialCommandGroup(new CmdExtendIntake(m_intake).withTimeout(0.1), new CmdOuttake(m_intake, m_hopper));

        //this shoot command is the ideal one with all capabilities
        shootCommand = new SequentialCommandGroup(
                        new InstantCommand(m_shooterLimelight::turnLEDOn),
                        new CmdRetractHopper(m_hopper).withTimeout(0.5), 
                        new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
                        // new CmdExtendIntake(m_intake),
                        new ParallelCommandGroup(
                            // new RunCommand(m_intake::runIntake, m_intake),
                            new CmdAlign(m_drive, m_shooterLimelight), 
                            new CmdHopperShooting(m_hopper, m_shooter::isReady),
                            new CmdShootDist(m_shooter, m_hood, m_shooterLimelight)
                        )
        );

        //use this shoot command for testing
        manualShoot = new SequentialCommandGroup(
                        new InstantCommand(m_shooterLimelight::turnLEDOn), 
                        new CmdRetractHopper(m_hopper).withTimeout(0.5),
                        new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
                        new ParallelCommandGroup(
                            new CmdHopperShooting(m_hopper, m_shooter::isReady),
                            new CmdShootDist(m_shooter, m_hood, m_shooterLimelight)
                        )
        );

        lowerHubShoot = new SequentialCommandGroup(
                            new CmdRetractHopper(m_hopper).withTimeout(0.5),
                            new InstantCommand(() -> m_shooter.setState(ShooterState.LOWERHUB)),
                            // new CmdExtendIntake(m_intake),
                            new ParallelCommandGroup(
                                // new RunCommand(m_intake::runIntake, m_intake),
                                new RunCommand(m_drive::stop, m_drive),
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new InstantCommand(() -> m_hood.startPID(28)),
                                new CmdShootRPM(m_shooter, 1200))
        );


        //AUTONOMOUS ROUTINES

        auto_1Ball = new SequentialCommandGroup(
                            alignShootCmd(),

                            trajectoryCmd(Trajectories.driveBack30In)
        );

        auto_2Ball = new SequentialCommandGroup(
                            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),

                            new ParallelDeadlineGroup(
                                trajectoryCmd(Trajectories.twoBallTraj), 
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            new CmdInPlaceTurn(m_drive, 180),

                            alignShootCmd()
        );

        auto_3Ball180 = new SequentialCommandGroup(

            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("3Ballv2_i.wpilib.json"), 
                new CmdExtendIntakeAndRun(m_intake, m_hopper)
            ),

            new CmdInPlaceTurn(m_drive, 180),
            alignShootCmd(),

            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("3Ballv2_ii.wpilib.json"),
                new CmdExtendIntakeAndRun(m_intake, m_hopper)
            ),

            turnRightToAligned(),
            alignShootCmd()
        );

        auto_S2H1 = new SequentialCommandGroup(

                            //drive and intake ball
                            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_i.wpilib.json"),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            //turn and shoot
                            new CmdInPlaceTurn(m_drive, 180),
                            alignShootCmd(),

                            //turn and hoard first ball
                            new CmdInPlaceTurn(m_drive, 90),
                            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_ii.wpilib.json"),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            //drive behind hub
                            new CmdInPlaceTurn(m_drive, -90),
                            trajectoryCmd("S2H1.wpilib.json"),

                            //outtake balls behind hub
                            new CmdExtendIntake(m_intake),
                            new CmdOuttake(m_intake, m_hopper, 0.5).withTimeout(2)

        );   

        auto_S2H2 = new SequentialCommandGroup(

                            //drive and intake ball
                            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_i.wpilib.json"),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            //turn and shoot
                            new CmdInPlaceTurn(m_drive, 180),
                            alignShootCmd(),

                            //turn and hoard first ball
                            new CmdInPlaceTurn(m_drive, 90),
                            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_ii.wpilib.json"),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            // turn and hoard second ball
                            new CmdInPlaceTurn(m_drive, 180),
                            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_iii.wpilib.json"), 
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)),

                            // trajectoryCmd(12),
                            // new ParallelDeadlineGroup(
                            //     trajectoryCmd(13),
                            //     new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            // ),
                            
                            //hide ball behinde hub
                            trajectoryCmd("S2H2_iv.wpilib.json"),

                            // new CmdInPlaceTurn(m_drive, 130),
                            new CmdExtendIntake(m_intake),
                            new CmdOuttake(m_intake, m_hopper, 0.4).withTimeout(1)

        );

        auto_4Ball180 = new SequentialCommandGroup(
                            //drive and intake 1 ball
                            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("4Ball_Terminal180_i.wpilib.json"),  
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)),

                            //turn and shoot 2 balls
                            new CmdInPlaceTurn(m_drive, 180),
                            shootCmd(),

                            //drive to ball and terminal and intake
                            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("4Ball_Terminal180_ii.wpilib.json"), 
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)),
                            new CmdExtendIntakeAndRun(m_intake, m_hopper).withTimeout(1),

                            //drive to tarmac and shoot
                            trajectoryCmd("Terminal2Tarmac.wpilib.json"),
                            new CmdInPlaceTurn(m_drive, 180),
                            alignShootCmd()

        );

        auto_5Ball180 = new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("3Ballv2_i.wpilib.json"), 
                new CmdExtendIntakeAndRun(m_intake, m_hopper)
            ),

            new CmdInPlaceTurn(m_drive, 180),
            alignShootCmd(),

            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("3Ballv2_ii.wpilib.json"),
                new CmdExtendIntakeAndRun(m_intake, m_hopper)
            ),

            new ParallelCommandGroup(
                turnRightToAligned(),
                shootCmd()
            ),

            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    trajectoryCmd("5Ballv2_i.wpilib.json"),
                    new InstantCommand(() -> m_drive.stop())),
                    // new WaitCommand(0.5)),
                new CmdExtendIntakeAndRun(m_intake, m_hopper)
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
                                new CmdExtendIntake(m_intake),
                                new CmdOuttake(m_intake, m_hopper, 0.85).withTimeout(1.5)
                            ).withTimeout(2),

                            new CmdInPlaceTurn(m_drive, 70),
                            
                            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("Billiards_i.wpilib.json"),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            new CmdInPlaceTurn(m_drive, 55),

                            new CmdExtendIntake(m_intake),
                            new CmdOuttake(m_intake, m_hopper, 0.6).withTimeout(1.5),

                            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("Billiards_ii.wpilib.json"),
                                new CmdExtendIntakeAndRun(m_intake, m_hopper)
                            ),

                            new CmdInPlaceTurn(m_drive, 96),

                            alignShootCmd()
        );

        auto_S1H1 = new SequentialCommandGroup(

            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("S1H1_i.wpilib.json"), 
                new CmdExtendIntakeAndRun(m_intake, m_hopper)  
            ),
            new InstantCommand(() -> m_drive.stop()),

            new CmdInPlaceTurn(m_drive, 180),
            
            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(m_shooterLimelight::turnLEDOn),
            new ParallelCommandGroup(
                new CmdAlign(m_drive, m_shooterLimelight),
                new CmdHopperShooting(m_hopper, m_shooter::isReady, 0.6),
                new CmdShootSingleBall(m_shooter, m_hood, m_shooterLimelight)
            ).withTimeout(2),
            new InstantCommand(m_shooterLimelight::turnLEDOff),
            
            trajectoryCmd("S1H1_ii.wpilib.json"),
            new CmdExtendIntake(m_intake),
            new CmdOuttake(m_intake, m_hopper, 0.5).withTimeout(1)
        );

        // auto_S1H1 = new SequentialCommandGroup(
        //     alignShootCmd(),

        //     new CmdInPlaceTurn(m_drive, 180),

        //     new ParallelDeadlineGroup(
        //         trajectoryCmd("S1H1_i.wpilib.json"), 
        //         new CmdExtendIntakeAndRun(m_intake, m_hopper)
        //     ),

        //     new InstantCommand(() -> m_drive.stop()),

        //     new CmdInPlaceTurn(m_drive, 180),

        //     trajectoryCmd("S1H1_ii.wpilib.json"),
        //     new InstantCommand(() -> m_drive.stop()),

        //     new CmdExtendIntake(m_intake),
        //     new CmdOuttake(m_intake, m_hopper, 0.5).withTimeout(2)
        // );

        auto_S1I1 = new SequentialCommandGroup(

            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("S1H1_i.wpilib.json"), 
                new CmdExtendIntakeAndRun(m_intake, m_hopper)  
            ),
            new InstantCommand(() -> m_drive.stop()),

            new CmdInPlaceTurn(m_drive, 180),
            
            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(m_shooterLimelight::turnLEDOn),
            new ParallelCommandGroup(
                new CmdAlign(m_drive, m_shooterLimelight),
                new CmdHopperShooting(m_hopper, m_shooter::isReady, 0.6),
                new CmdShootSingleBall(m_shooter, m_hood, m_shooterLimelight)
            ).withTimeout(2),
            new InstantCommand(m_shooterLimelight::turnLEDOff)
        );

        // auto_S1I1 = new SequentialCommandGroup(
        //     alignShootCmd(),
            
        //     new CmdInPlaceTurn(m_drive, 180),

        //     new ParallelDeadlineGroup(
        //         trajectoryCmd("S1H1_i.wpilib.json"),
        //         new CmdExtendIntakeAndRun(m_intake, m_hopper)
        //     )
        // );

        auto_S1H2 = new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("S1H1_i.wpilib.json"), 
                new CmdExtendIntakeAndRun(m_intake, m_hopper)  
            ),
            new InstantCommand(() -> m_drive.stop()),

            new CmdInPlaceTurn(m_drive, 180),
            
            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(m_shooterLimelight::turnLEDOn),
            new ParallelCommandGroup(
                new CmdAlign(m_drive, m_shooterLimelight),
                new CmdHopperShooting(m_hopper, m_shooter::isReady, 0.6),
                new CmdShootSingleBall(m_shooter, m_hood, m_shooterLimelight)
            ).withTimeout(2),
            new InstantCommand(m_shooterLimelight::turnLEDOff),

            new InstantCommand(() -> m_intake.ejectIntake(), m_intake),
            new ParallelDeadlineGroup(
                trajectoryCmd("S1H2_ii.wpilib.json"), 
                new CmdExtendIntakeAndRun(m_intake, m_hopper)  
            ),

            new CmdInPlaceTurn(m_drive, 180),

            trajectoryCmd("S1H2_iii.wpilib.json"), 
            
            new CmdExtendIntake(m_intake),
            new CmdOuttake(m_intake, m_hopper, 0.5).withTimeout(2)
        );

        // auto_S1H2 = new SequentialCommandGroup(
        //     alignShootCmd(),
            
        //     new CmdInPlaceTurn(m_drive, 180),

        //     new ParallelDeadlineGroup(
        //         trajectoryCmd("S1H1_i.wpilib.json"), 
        //         new CmdExtendIntakeAndRun(m_intake, m_hopper)  
        //     ),

        //     new CmdInPlaceTurn(m_drive, 180),

        //     new ParallelDeadlineGroup(
        //         trajectoryCmd("S1H2_ii.wpilib.json"), 
        //         new CmdExtendIntakeAndRun(m_intake, m_hopper)  
        //     ),

        //     new CmdInPlaceTurn(m_drive, 180),
        //     trajectoryCmd("S1H2_iii.wpilib.json"),
        //     new InstantCommand(() -> m_drive.stop()),
            
        //     new CmdExtendIntake(m_intake),
        //     new CmdOuttake(m_intake, m_hopper, 0.5).withTimeout(2)
        // );

        // Setup auto-selector
        NarwhalDashboard.addAuto("1 Ball", auto_1Ball);
        NarwhalDashboard.addAuto("2 Ball", auto_2Ball);
        NarwhalDashboard.addAuto("S2H2", auto_S2H2);
        NarwhalDashboard.addAuto("S2H1", auto_S2H1);
        NarwhalDashboard.addAuto("3 Ball", auto_3Ball180);
        NarwhalDashboard.addAuto("4 Ball 180 Terminal", auto_4Ball180);
        NarwhalDashboard.addAuto("5 Ball 180 Terminal", auto_5Ball180);
        NarwhalDashboard.addAuto("Billiards", auto_Billiards);
        NarwhalDashboard.addAuto("S1H1", auto_S1H1);
        NarwhalDashboard.addAuto("S1, intake 1 enemy", auto_S1I1);
        NarwhalDashboard.addAuto("S1H2", auto_S1H2);
    }

    private RamseteCommand trajectoryCmd(String fileName) {
        return trajectoryCmd(trajectoryMap.get(fileName));
    }

    private RamseteCommand trajectoryCmd(Trajectory traj) {
        return new RamseteCommand(traj, 
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

    private SequentialCommandGroup shootCmd() {
        return new SequentialCommandGroup(
            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(() -> m_shooterLimelight.turnLEDOn()),
            new ParallelCommandGroup(
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootDist(m_shooter, m_hood, m_shooterLimelight)
            ).withTimeout(2)
        );
    }
    
    private SequentialCommandGroup shootCmd(int RPM) {
        return new SequentialCommandGroup(
            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(() -> m_shooterLimelight.turnLEDOn()),
            new ParallelCommandGroup(
                new InstantCommand(() -> m_hood.startPID(HoodConstants.HOME_ANGLE)),
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootRPM(m_shooter, RPM)
            ).withTimeout(2)
        );
    }

    private SequentialCommandGroup shootCmd(int RPM, double angle) {
        return new SequentialCommandGroup(
            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new ParallelCommandGroup(
                new InstantCommand(() -> m_hood.startPID(angle)),
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootRPM(m_shooter, RPM)
            ).withTimeout(2)
        );
    }

    private SequentialCommandGroup alignShootCmd() {
        return new SequentialCommandGroup(
            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(m_shooterLimelight::turnLEDOn),
            new ParallelCommandGroup(
                new CmdAlign(m_drive, m_shooterLimelight),
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootDist(m_shooter, m_hood, m_shooterLimelight)
            ).withTimeout(2),
            new InstantCommand(m_shooterLimelight::turnLEDOff)
        );
    }

    private Command turnLeftToAligned() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_shooterLimelight.turnLEDOn()),
            new CmdInPlaceTurn(m_drive, 170).until(m_shooterLimelight::hasValidTarget),
            new CmdAlign(m_drive, m_shooterLimelight)
        );
    }

    private Command turnRightToAligned() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_shooterLimelight.turnLEDOn()),
            new CmdInPlaceTurn(m_drive, -170).until(m_shooterLimelight::hasValidTarget),
            new CmdAlign(m_drive, m_shooterLimelight)
        ).withTimeout(3);
    }

    /**
     * @return a Pose2d with the same rotation and reversed (unary minus) rotation
     */ 
    private Pose2d reversedPose(Pose2d pose) {
        return new Pose2d(
            pose.getTranslation(),
            pose.getRotation().unaryMinus()
        );
    }

    private void initDashboard() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            SmartDashboard.putData("Drivetrain", m_drive);
            SmartDashboard.putData("Intake", m_intake);
            SmartDashboard.putData("Hopper", m_hopper);
            SmartDashboard.putData("Climber", m_climber);
            SmartDashboard.putData("Shooter", (PIDSubsystem)m_shooter);
            SmartDashboard.putData("Hood", (PIDSubsystem)m_hood);
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
            //ll.turnLEDOn();
        }
    }

    public void updateDashboard() {

        // Update necessary Nardash debug data

        NarwhalDashboard.put("time", Timer.getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("rpm", m_shooter.getMeasurement());
        NarwhalDashboard.put("range", m_shooterLimelight.calculateDistToTopTarget(Constants.VisionConstants.TARGET_HEIGHT));
        NarwhalDashboard.put("x", m_drive.getPose().getX());
        NarwhalDashboard.put("y", m_drive.getPose().getY());
        NarwhalDashboard.put("theta", Units.degreesToRadians(m_drive.getHeading()));
        NarwhalDashboard.put("climbEnc", m_climber.getCurrentTicksLeft());

        // Post miscellaneous other debug data to Smartdash

        SmartDashboard.putNumber("distance to the limelight", m_shooterLimelight.calculateDistToTopTarget(Constants.VisionConstants.TARGET_HEIGHT));
        SmartDashboard.putNumber("ty", m_shooterLimelight.getValue(LimelightKey.VERTICAL_OFFSET, 2));
        SmartDashboard.putNumber("adjusted ty", m_shooterLimelight.getValue(LimelightKey.VERTICAL_OFFSET, 5) * (2/3));

        // SmartDashboard.putBoolean("Shooter is ready", m_shooter.isReady());
        SmartDashboard.putString("Shooter state", m_shooter.getState().toString());
        SmartDashboard.putNumber("Shooter Setpoint", m_shooter.getSetpoint());
        SmartDashboard.putNumber("Shooter RPM", m_shooter.getMeasurement());

        SmartDashboard.putNumber("Hood Setpoint", m_hood.getSetpoint());
        SmartDashboard.putNumber("Hood angle", m_hood.getMeasurement());

        SmartDashboard.putString("Intake state:", m_intake.getSolenoid());

        // SmartDashboard.putString("Drive half speed", String.valueOf(driveHalfSpeed));
        SmartDashboard.putNumber("ratio", m_shooter.ratio);
    }

    public Command getAutonomousCommand() {
        
        // I don't understand why putting this in the constructor or initAutos doesn't work, so I put it here and it works (in sim)
        initialPoses.put(auto_1Ball, Trajectories.driveBack30In.getInitialPose());
        initialPoses.put(auto_2Ball, Trajectories.twoBallTraj.getInitialPose());
        initialPoses.put(auto_3Ball180, trajectoryMap.get("3Ballv2_i.wpilib.json").getInitialPose());
        initialPoses.put(auto_S2H1, trajectoryMap.get("S2H2_i.wpilib.json").getInitialPose());
        initialPoses.put(auto_S2H2, trajectoryMap.get("S2H2_i.wpilib.json").getInitialPose());
        initialPoses.put(auto_4Ball180, trajectoryMap.get("4Ball_Terminal180_i.wpilib.json").getInitialPose());
        initialPoses.put(auto_5Ball180, trajectoryMap.get("3Ballv2_i.wpilib.json").getInitialPose());
        initialPoses.put(auto_Billiards, new Pose2d(6.8, 6.272, Rotation2d.fromDegrees(45)));
        initialPoses.put(auto_S1H1, trajectoryMap.get("S1H1_i.wpilib.json").getInitialPose());
        initialPoses.put(auto_S1I1, trajectoryMap.get("S1H1_i.wpilib.json").getInitialPose());
        initialPoses.put(auto_S1H2, trajectoryMap.get("S1H1_i.wpilib.json").getInitialPose());

        Command selectedAuto = NarwhalDashboard.getSelectedAuto();

        if (selectedAuto == null) {
            return null;
        }

        m_drive.resetPose(initialPoses.get(selectedAuto));
        return selectedAuto;

    }

    public void initPneumatics() {
        m_climber.retractPiston();
        m_intake.retractIntake();
    }

}