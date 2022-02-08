package frc.team3128;

import java.io.IOException;
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
import frc.team3128.common.hardware.limelight.LimelightKey;
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
    // private Climber m_climber;

    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

    private Limelight m_shooterLimelight;
    private Limelight m_balLimelight;

    private String[] trajJson = {"paths/2_BallBot_i.wpilib.json", 
                                "paths/2_BallFar_i.wpilib.json", 
                                "paths/2_BallFar_ii.wpilib.json", 
                                "paths/2_BallFar_iii.wpilib.json", 
                                "paths/2_BallMid_i.wpilib.json", // haha mid
                                "paths/2_BallTop_i.wpilib.json", 
                                "paths/3_Ball_i.wpilib.json", 
                                "paths/3_Ball_ii.wpilib.json", 
                                "paths/3_Ball_iii.wpilib.json", 
                                "paths/3_Ball_iv.wpilib.json", 
                                "paths/4_Ball_i.wpilib.json", 
                                "paths/4_Ball_ii.wpilib.json"};
    private Trajectory[] trajectory = new Trajectory[trajJson.length];

    
    private CmdIntakeCargo intakeCargoCommand;
    private SequentialCommandGroup extendIntakeAndRun;
    private CmdRetractHopper retractHopperCommand;
    private Command shootCommand;
    private CmdShootRPM manualShoot;
    private SequentialCommandGroup shootCommand2;
    private CmdClimb climbCommand;

    private HashMap<Command, Pose2d> initialPoses;
    private Command auto_2balltop;
    private Command auto_3ball;

    private boolean DEBUG = true;

    public RobotContainer() {
        
        m_drive = NAR_Drivetrain.getInstance();
        m_shooter = Shooter.getInstance();
        m_intake = Intake.getInstance();
        m_hopper = Hopper.getInstance();
        // m_climber = Climber.getInstance();

        //Enable all PIDSubsystems so that useOutput runs
        m_shooter.enable();

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);

        m_shooterLimelight = new Limelight("limelight-pog", VisionConstants.TOP_CAMERA_ANGLE, 
                                                            VisionConstants.TOP_CAMERA_HEIGHT, 
                                                            VisionConstants.TOP_FRONT_DIST, 0); 
        m_balLimelight = new Limelight("limelight-sog", VisionConstants.BALL_LL_ANGLE, 
                                                        VisionConstants.BALL_LL_HEIGHT, 
                                                        VisionConstants.BALL_LL_FRONT_DIST, 0);

        m_commandScheduler.setDefaultCommand(m_drive, new CmdArcadeDrive(m_drive, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle));
        m_commandScheduler.setDefaultCommand(m_hopper, new CmdHopperDefault(m_hopper, m_shooter::isReady)); //TODO: make input into this good method ???

        initialPoses = new HashMap<Command, Pose2d>();
        initialPoses.put(auto_2balltop, trajectory[5].getInitialPose());
        initialPoses.put(auto_3ball, trajectory[6].getInitialPose());

        try {
            for (int i = 0; i < trajJson.length; i++) {
                // Get a path from the string specified in trajJson, and load it into trajectory[i]
                trajectory[i] = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajJson[i]));
            }
        } catch (IOException ex) {
            DriverStation.reportError("IOException opening trajectory:", ex.getStackTrace());
        }

        initAutos();
        configureButtonBindings();
        dashboardInit();
        
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
        // 10: make climb go up a bit
        // 11: extend climber piston
        // 12: retract climber piston
        // 15: push climber all the way to top magnet
        // 14: push climber all the way to bottom magnet

        m_rightStick.getButton(1).whenHeld(new SequentialCommandGroup(
                                            new CmdExtendIntake(m_intake), intakeCargoCommand))
                                .whenReleased(retractHopperCommand);
        
        m_rightStick.getButton(2).whenPressed(shootCommand2) //manualShoot
                                .whenReleased(new InstantCommand(m_shooter::stopShoot,m_shooter));

        m_rightStick.getButton(3).whenPressed(retractHopperCommand);

        m_rightStick.getButton(5).whenPressed(new SequentialCommandGroup(new InstantCommand(m_intake::ejectIntake, m_intake), new RunCommand(m_intake::runIntakeBack, m_intake)).withTimeout(0.15))
                                .whenReleased(new InstantCommand(m_intake::stopIntake, m_intake));
        m_rightStick.getButton(6).whenPressed(m_intake::retractIntake, m_intake);

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

        retractHopperCommand = new CmdRetractHopper(m_hopper);
        // climbCommand = new CmdClimb(m_climber);
        
        intakeCargoCommand = new CmdIntakeCargo(m_intake, m_hopper);

        extendIntakeAndRun = new SequentialCommandGroup(new CmdExtendIntake(m_intake), intakeCargoCommand);

        //this shoot command is the ideal one with all capabilities
        shootCommand = new SequentialCommandGroup(
                          new CmdRetractHopper(m_hopper), 
                          new ParallelCommandGroup(new CmdAlign(m_drive, m_shooterLimelight), 
                          new CmdShootRPM(m_shooter, m_shooter.calculateMotorVelocityFromDist(m_shooterLimelight.getYPrime(VisionConstants.TARGET_HEIGHT, VisionConstants.SAMPLE_RATE)))));

        //use this shoot command for testing
        shootCommand2 = new SequentialCommandGroup(new CmdRetractHopper(m_hopper),  
                          new CmdShootRPM(m_shooter, 3000));
        manualShoot = new CmdShootRPM(m_shooter, 3000);


        auto_2balltop = new ParallelCommandGroup(
            extendIntakeAndRun,
            new SequentialCommandGroup(trajRamsete(5),
                                        new InstantCommand(m_drive::stop, m_drive),
                                        shootCommand2)
        );

        auto_3ball = new SequentialCommandGroup(
            shootCommand2.withTimeout(4), // Edit this timeout when tested
            new ParallelCommandGroup(extendIntakeAndRun,
                new SequentialCommandGroup(
                    trajRamsete(6),
                    trajRamsete(7),
                    trajRamsete(8),
                    trajRamsete(9),
                    shootCommand2.withTimeout(4)
                ))
        );

        // Setup auto-selector
        NarwhalDashboard.addAuto("Basic Auto", auto_2balltop);
        // NarwhalDashboard.addAuto("Ball Pursuit", cmdBallPursuit);
    }

    // Helper for initAutos so we don't clog it up with all of these params
    private RamseteCommand trajRamsete(int i) {
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

    private void dashboardInit() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            SmartDashboard.putData("Drivetrain", m_drive);
            SmartDashboard.putData("Intake", m_intake);
            SmartDashboard.putData("Hopper", m_hopper);
            SmartDashboard.putData("Shooter", m_shooter);
            SmartDashboard.putBoolean("Shooter at Setpoint", m_shooter.isReady());
            SmartDashboard.putString("Shooter state", m_shooter.getState().toString());
            SmartDashboard.putNumber("Shooter Setpoint", m_shooter.getSetpoint());
        }

        NarwhalDashboard.startServer();
        setupLimelights(m_shooterLimelight, m_balLimelight); 
            
    }

    public void stopDrivetrain() {
        m_drive.stop();
    }
  
    private void setupLimelights(Limelight... limelightList) {
        Log.info("NarwhalRobot", "Setting up limelight chooser...");
      
        for (Limelight ll : limelightList) {
            NarwhalDashboard.addLimelight(ll);
        }
    }
  
    public void updateDashboard() {
        NarwhalDashboard.put("time", Timer.getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("rpm", m_shooter.getMeasurement());
        NarwhalDashboard.put("range", m_shooterLimelight.calculateYPrimeFromTY(m_shooterLimelight.getValue(LimelightKey.VERTICAL_OFFSET, VisionConstants.SAMPLE_RATE) * Math.PI / 180, VisionConstants.TARGET_HEIGHT));
    }

    public Command getAutonomousCommand() {
        // TODO: MAKE HASHMAP CONTAINING INITIAL POSE2D AND AUTO
        m_drive.resetPose(initialPoses.get(auto_2balltop));
        // return NarwhalDashboard.getSelectedAuto();
        return auto_2balltop;
    }

}
