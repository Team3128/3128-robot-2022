package frc.team3128;

import java.io.IOException;
import java.nio.file.Path;
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
import frc.team3128.commands.*;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.*;

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
    private Limelight m_balLimelight;

    private String[] trajJson = {"paths/2_BallBot_i.wpilib.json", //0
                                "paths/2_BallMid_i.wpilib.json", //1
                                "paths/2_BallTop_i.wpilib.json", //2
                                "paths/3_Ball_i.wpilib.json", //3
                                "paths/3_Ball_ii.wpilib.json", //4
                                "paths/3_BallTerm_i.wpilib.json", //5
                                "paths/3_BallTerm_ii.wpilib.json", //6
                                "paths/3_BallTerm_iii.wpilib.json", //7
                                "paths/3_BallTerm_iv.wpilib.json", //8
                                "paths/3_BallHK_i.wpilib.json", //9 
                                "paths/3_BallHK_ii.wpilib.json", //10    
                                "paths/4_BallE_i.wpilib.json", //11 
                                "paths/4_BallE_ii.wpilib.json", //12
                                "paths/4_Ball_i.wpilib.json", //13
                                "paths/4_Ball_ii.wpilib.json" //14
                                };
    private Trajectory[] trajectory = new Trajectory[trajJson.length];

    
    private CmdIntakeCargo intakeCargoCommand;
    private SequentialCommandGroup extendIntakeAndRun;
    private CmdRetractHopper retractHopperCommand;
    private Command shootCommand;
    private SequentialCommandGroup manualShoot;
    private SequentialCommandGroup lowerHubShoot;
    private SequentialCommandGroup shootCommand2;
    private CmdClimb climbCommand;

    private HashMap<Command, Pose2d> initialPoses;

    private Command auto_2BallTop;
    private Command auto_2BallMid;
    private Command auto_2BallBot;
    private Command auto_3BallTerminal;
    private Command auto_3BallHook;
    private Command auto_3BallHersheyKiss;
    private Command auto_4BallE;


    private boolean DEBUG = true;

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
        m_balLimelight = new Limelight("limelight-sog", VisionConstants.BALL_LL_ANGLE, 
                                                        VisionConstants.BALL_LL_HEIGHT, 
                                                        VisionConstants.BALL_LL_FRONT_DIST, 0);

        m_commandScheduler.setDefaultCommand(m_drive, new CmdArcadeDrive(m_drive, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle));
        //m_commandScheduler.setDefaultCommand(m_hopper, new CmdHopperDefault(m_hopper, m_shooter::isReady)); //TODO: make input into this good method ???


        initAutos();
        initDashboard();
        initLimelights(m_shooterLimelight, m_balLimelight); 
        configureButtonBindings();
        
        if(RobotBase.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        // Buttons...
        // right:
        // 1 (trigger): intake 
        // 2: shoot
        // 8: climb
        // 9: stop climb
        //
        // left:
        // 9: make climber go up
        // 10: make climb go down
        // 11: extend climber piston
        // 12: retract climber piston
        // 15: push climber all the way to top magnet
        // 14: push climber all the way to bottom magnet

        //RIGHT
        m_rightStick.getButton(1).whenHeld(extendIntakeAndRun);
                                // .whenReleased(retractHopperCommand); Garrison said no to this
        
        m_rightStick.getButton(2).whenPressed(shootCommand)
                                .whenReleased(new ParallelCommandGroup(new InstantCommand(m_shooter::stopShoot,m_shooter), new InstantCommand(m_shooterLimelight::turnLEDOff)));

        m_rightStick.getButton(3).whenPressed(retractHopperCommand);

        m_rightStick.getButton(5).whenPressed(new SequentialCommandGroup(new InstantCommand(m_intake::ejectIntake, m_intake), new RunCommand(m_intake::runIntakeBack, m_intake)).withTimeout(0.15))
                                .whenReleased(new InstantCommand(m_intake::stopIntake, m_intake));

        m_rightStick.getButton(6).whenPressed(m_intake::retractIntake, m_intake);

        m_rightStick.getButton(11).whenPressed(manualShoot) //manualShoot
                                .whenReleased(new ParallelCommandGroup(new InstantCommand(m_shooter::stopShoot,m_shooter), new InstantCommand(m_shooterLimelight::turnLEDOff)));

        //LEFT
        m_leftStick.getButton(1).whenHeld(lowerHubShoot);

        m_leftStick.getButton(9).whenPressed(new InstantCommand(m_climber::extendBoth, m_climber))
                                .whenReleased(new InstantCommand(m_climber::stopBoth, m_climber));

        m_leftStick.getButton(10).whenPressed(new InstantCommand(m_climber::retractBoth, m_climber))
                                .whenReleased(new InstantCommand(m_climber::stopBoth, m_climber));

        m_leftStick.getButton(11).whenPressed(new InstantCommand(m_climber::extendPiston, m_climber));
        m_leftStick.getButton(12).whenPressed(new InstantCommand(m_climber::retractPiston, m_climber));
        m_leftStick.getButton(16).whenPressed(new InstantCommand(m_climber::engageBreak, m_climber));
        m_leftStick.getButton(15).whenPressed(new InstantCommand(m_climber::disengageBreak, m_climber));

        


        //climber buttons (uncomment when testing climber)
        // m_leftStick.getButton(10).whenPressed(new CmdClimbEncoder(m_climber, ClimberConstants.SMALL_VERTICAL_DISTANCE));
        // m_leftStick.getButton(11).whenPressed(new InstantCommand(m_climber::extendArm, m_climber));
        // m_leftStick.getButton(12).whenPressed(new InstantCommand(m_climber::retractArm, m_climber));
        // m_leftStick.getButton(15).whenPressed(new CmdClimbExtend(m_climber));
        // m_leftStick.getButton(14).whenPressed(new CmdClimbRetract(m_climber));

        //m_rightStick.getButton(8).whenPressed(climbCommand);
        //m_rightStick.getButton(8).whenReleased(new InstantCommand(m_climber::climberStop, m_climber));
        //m_rightStick.getButton(9).whenPressed(new InstantCommand(m_climber::climberStop, m_climber));
    }


    private void initAutos() {

        try {
            for (int i = 0; i < trajJson.length; i++) {
                // Get a path from the string specified in trajJson, and load it into trajectory[i]
                Path path = Filesystem.getDeployDirectory().toPath().resolve(trajJson[i]);
                trajectory[i] = TrajectoryUtil.fromPathweaverJson(path);
            }
        } catch (IOException ex) {
            DriverStation.reportError("IOException opening trajectory:", ex.getStackTrace());
        }

        initialPoses = new HashMap<Command, Pose2d>();

        retractHopperCommand = new CmdRetractHopper(m_hopper);
        // climbCommand = new CmdClimb(m_climber);
        
        intakeCargoCommand = new CmdIntakeCargo(m_intake, m_hopper);

        extendIntakeAndRun = new SequentialCommandGroup(new CmdExtendIntake(m_intake).withTimeout(0.1), intakeCargoCommand);

        //this shoot command is the ideal one with all capabilities
        shootCommand = new SequentialCommandGroup(
                        new InstantCommand(m_shooterLimelight::turnLEDOn),
                        new CmdRetractHopper(m_hopper), 
                        new ParallelCommandGroup(
                            new CmdAlign(m_drive, m_shooterLimelight), 
                            new CmdHopperShooting(m_hopper, m_shooter::isReady),
                            new CmdShootRPM(m_shooter, m_shooter.calculateMotorVelocityFromDist(m_shooterLimelight.calculateDistToTopTarget(Constants.VisionConstants.TARGET_HEIGHT))))
        );

        //use this shoot command for testing
        manualShoot = new SequentialCommandGroup(
                        new InstantCommand(m_shooterLimelight::turnLEDOn), 
                        new CmdRetractHopper(m_hopper),
                        new ParallelCommandGroup(
                            new CmdHopperShooting(m_hopper, m_shooter::isReady),
                            new CmdShootRPM(m_shooter, m_shooter.calculateMotorVelocityFromDist(m_shooterLimelight.calculateDistToTopTarget(Constants.VisionConstants.TARGET_HEIGHT))))
        );

        lowerHubShoot = new SequentialCommandGroup(
                            new CmdRetractHopper(m_hopper),
                            new ParallelCommandGroup(
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter,1250))
        );
                        
        shootCommand2 = new SequentialCommandGroup(
                            new CmdRetractHopper(m_hopper),  
                            new ParallelCommandGroup(
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter,3000))
        );


        //AUTONOMOUS ROUTINES
        auto_2BallBot = new SequentialCommandGroup(

                            new CmdExtendIntake(m_intake).withTimeout(0.1),

                            new ParallelDeadlineGroup(
                                trajectoryCmd(0).andThen(m_drive::stop, m_drive),

                                new InstantCommand(() -> {
                                m_intake.runIntake();
                                m_hopper.runHopper();
                                }, m_intake, m_hopper)
                            ),

                            new CmdRetractHopper(m_hopper).withTimeout(0.5),
                            new ParallelCommandGroup(
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter, 3000)
                            ).withTimeout(4)

        );
        
        auto_2BallMid = new SequentialCommandGroup(

                            new CmdExtendIntake(m_intake).withTimeout(0.1),
                            
                            new ParallelDeadlineGroup(
                                trajectoryCmd(1).andThen(m_drive::stop, m_drive),

                                new InstantCommand(() -> {
                                    m_intake.runIntake();
                                    m_hopper.runHopper();
                                }, m_intake, m_hopper)
                            ),

                            new CmdRetractHopper(m_hopper).withTimeout(0.5),
                            new ParallelCommandGroup(
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter, 3500)
                            ).withTimeout(4)

        );

        auto_2BallTop = new SequentialCommandGroup(

                            new CmdExtendIntake(m_intake).withTimeout(0.1),
                                            
                            new ParallelDeadlineGroup(
                                trajectoryCmd(2).andThen(m_drive::stop, m_drive),

                                new InstantCommand(() -> {
                                    m_intake.runIntake();
                                    m_hopper.runHopper();
                                }, m_intake, m_hopper)
                            ),

                            new CmdRetractHopper(m_hopper).withTimeout(0.5),
                            new ParallelCommandGroup(
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter, 3000)
                            ).withTimeout(4)

        );

        auto_3BallHook = new SequentialCommandGroup(

            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new ParallelCommandGroup(
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootRPM(m_shooter, 3350)
            ).withTimeout(3),

            new CmdExtendIntake(m_intake).withTimeout(0.1),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    trajectoryCmd(3),
                    trajectoryCmd(4),
                    new InstantCommand(m_drive::stop, m_drive)
                ),
                new InstantCommand(() -> {
                    m_intake.runIntake();
                    m_hopper.runHopper();
                }, m_intake, m_hopper)
            ),

            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new ParallelCommandGroup(
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootRPM(m_shooter, 3250)
            ).withTimeout(3)

        );
        
        auto_3BallTerminal = new SequentialCommandGroup(

                            new CmdRetractHopper(m_hopper).withTimeout(0.5),
                            new ParallelCommandGroup(
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter, 3000)
                            ).withTimeout(4), // Edit this timeout when tested

                            new CmdExtendIntake(m_intake).withTimeout(0.1),
                            new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                    trajectoryCmd(5),
                                    trajectoryCmd(6),
                                    trajectoryCmd(7),
                                    trajectoryCmd(8),
                                    new InstantCommand(m_drive::stop, m_drive)
                                ),
                                new InstantCommand(() -> {
                                    m_intake.runIntake();
                                    m_hopper.runHopper();
                                }, m_intake, m_hopper)
                            ),

                            new CmdRetractHopper(m_hopper).withTimeout(0.5),
                            new ParallelCommandGroup(
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter, 3000)
                            ).withTimeout(4)

        );

        auto_3BallHersheyKiss = new SequentialCommandGroup(
                            new CmdRetractHopper(m_hopper).withTimeout(0.5),
                            new ParallelCommandGroup(
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter, 3000)
                            ).withTimeout(4),
                            
                            new CmdExtendIntake(m_intake).withTimeout(0.1),
                            new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                    trajectoryCmd(9),
                                    trajectoryCmd(10),
                                    new InstantCommand(m_drive::stop, m_drive)
                                ),
                                new InstantCommand(() -> {
                                    m_intake.runIntake();
                                    m_hopper.runHopper();
                                }, m_intake, m_hopper)
                            ),

                            new CmdRetractHopper(m_hopper).withTimeout(0.5),
                            new ParallelCommandGroup(
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter, 3000)
                            )
        );

        auto_4BallE = new SequentialCommandGroup(

                            new CmdExtendIntake(m_intake).withTimeout(0.1),
                            new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                    trajectoryCmd(11),
                                    new InstantCommand(m_drive::stop, m_drive)
                                ),
                                new InstantCommand(() -> {
                                    m_intake.runIntake();
                                    m_hopper.runHopper();
                                }, m_intake, m_hopper)
                            ),

                            new CmdRetractHopper(m_hopper).withTimeout(0.5),
                            new ParallelCommandGroup(
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter, 3000)
                            ),

                            new CmdExtendIntake(m_intake).withTimeout(0.1),
                            new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                    trajectoryCmd(12),
                                    new InstantCommand(m_drive::stop, m_drive)
                                ),
                                new InstantCommand(() -> {
                                    m_intake.runIntake();
                                    m_hopper.runHopper();
                                }, m_intake, m_hopper)
                            ),

                            new CmdRetractHopper(m_hopper).withTimeout(0.5),
                            new ParallelCommandGroup(
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter, 3000)
                            )

        );

        // Setup auto-selector
        NarwhalDashboard.addAuto("2 Ball Bottom", auto_2BallBot);
        NarwhalDashboard.addAuto("2 Ball Mid", auto_2BallMid);
        NarwhalDashboard.addAuto("2 Ball Top", auto_2BallTop);
        NarwhalDashboard.addAuto("3 Ball Hook", auto_3BallHook);
        NarwhalDashboard.addAuto("3 Ball Terminal", auto_3BallTerminal);
        NarwhalDashboard.addAuto("3 Ball Hershey Kiss", auto_3BallHersheyKiss);
        NarwhalDashboard.addAuto("4 Ball E", auto_4BallE);
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

    private void initDashboard() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            SmartDashboard.putData("Drivetrain", m_drive);
            SmartDashboard.putData("Intake", m_intake);
            SmartDashboard.putData("Hopper", m_hopper);
            SmartDashboard.putData("Shooter", m_shooter);
            SmartDashboard.putBoolean("Shooter at Setpoint", m_shooter.isReady());
            SmartDashboard.putString("Shooter state", m_shooter.getState().toString());
            SmartDashboard.putNumber("Shooter Setpoint", m_shooter.getSetpoint());
            SmartDashboard.putNumber("Shooter RPM", m_shooter.getMeasurement());
        }

        NarwhalDashboard.startServer();       
    }

    public void stopDrivetrain() {
        m_drive.stop();
    }
  
    private void initLimelights(Limelight... limelightList) {
        Log.info("NarwhalRobot", "Setting up limelight chooser...");
      
        for (Limelight ll : limelightList) {
            NarwhalDashboard.addLimelight(ll);
        }
    }
  
    public void updateDashboard() {
        NarwhalDashboard.put("time", Timer.getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("rpm", m_shooter.getMeasurement());
        NarwhalDashboard.put("range", m_shooterLimelight.calculateDistToTopTarget(Constants.VisionConstants.TARGET_HEIGHT));
        SmartDashboard.putNumber("range", m_shooterLimelight.calculateDistToTopTarget(Constants.VisionConstants.TARGET_HEIGHT));

        // SmartDashboard.putBoolean("Shooter at Setpoint", m_shooter.isReady());
        // SmartDashboard.putString("Shooter state", m_shooter.getState().toString());
        // SmartDashboard.putNumber("Shooter Setpoint", m_shooter.getSetpoint());
        // SmartDashboard.putNumber("Shooter RPM", m_shooter.getMeasurement());
    }

    public Command getAutonomousCommand() {
        
        // I don't understand why putting this in the constructor or initAutos doesn't work, so I put it here and it works (in sim)
        initialPoses.put(auto_2BallBot, trajectory[0].getInitialPose());
        initialPoses.put(auto_2BallMid, trajectory[1].getInitialPose());
        initialPoses.put(auto_2BallTop, trajectory[2].getInitialPose());
        initialPoses.put(auto_3BallHook, trajectory[3].getInitialPose());
        initialPoses.put(auto_3BallTerminal, trajectory[5].getInitialPose());
        initialPoses.put(auto_3BallHersheyKiss, trajectory[9].getInitialPose());
        initialPoses.put(auto_4BallE, trajectory[11].getInitialPose());

        m_drive.resetPose(initialPoses.get(NarwhalDashboard.getSelectedAuto()));
        return NarwhalDashboard.getSelectedAuto();

    }

}