package frc.team3128;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
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
    // private Climber m_climber;

    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

    private Limelight m_shooterLimelight;
    private Limelight m_balLimelight;

    private String trajJson = "paths/jude_path_o_doom.wpilib.json";
    private Trajectory trajectory = new Trajectory();

    private Command auto;
    private CmdIntakeCargo intakeCargoCommand;
    private CmdRetractHopper retractHopperCommand;
    private Command shootCommand;
    private CmdShoot manualShoot;
    private SequentialCommandGroup shootCommand2;
    private CmdClimb climbCommand;

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

        m_shooterLimelight = new Limelight("pog", 0, 0, 0, 0); // these are very fake right now
        m_balLimelight = new Limelight("sog", 0, 0, 0, 0);

        m_commandScheduler.setDefaultCommand(m_drive, new CmdArcadeDrive(m_drive, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle));
        m_commandScheduler.setDefaultCommand(m_hopper, new CmdHopperDefault(m_hopper, m_shooter::atSetpoint)); //TODO: make input into this good method ???

        try {
            Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(trajJson);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajPath);
        } catch (IOException ex) {
            DriverStation.reportError("Me me no open trajectory: " + trajJson, ex.getStackTrace());
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
        //TODO MAKE SURE whenHeld() works
        //m_rightStick.getButton(1).whenActive(new IntakeCargo(m_intake, m_hopper));
        //m_rightStick.getButton(1).whenReleased(new InstantCommand(m_intake::stopIntake, m_intake));

        m_rightStick.getButton(1).whenHeld( new SequentialCommandGroup(
                                            new SequentialCommandGroup(new InstantCommand(m_intake::ejectIntake, m_intake), new RunCommand(m_intake::runIntakeBack, m_intake)).withTimeout(0.15),
                                            intakeCargoCommand))
                                .whenReleased(retractHopperCommand);
        
        // m_rightStick.getButton(2).whenActive(new SequentialCommandGroup(new PrintCommand("button 2 active"), shootCommand));
        // m_rightStick.getButton(2).whenReleased(new InstantCommand(m_shooter::stopShoot, m_shooter));

        m_rightStick.getButton(3).whenPressed(retractHopperCommand);

        m_rightStick.getButton(5).whenPressed(new SequentialCommandGroup(new InstantCommand(m_intake::ejectIntake, m_intake), new RunCommand(m_intake::runIntakeBack, m_intake)).withTimeout(0.15))
                                .whenReleased(new InstantCommand(m_intake::stopIntake, m_intake));
        m_rightStick.getButton(6).whenPressed(m_intake::retractIntake, m_intake);

        m_rightStick.getButton(2).whenHeld(shootCommand2);

        // m_rightStick.getButton(2).whenHeld(shootCommand);

        //m_rightStick.getButton(8).whenActive(climbCommand);
        //m_rightStick.getButton(8).whenReleased(new InstantCommand(m_climber::climberStop, m_climber));
        //m_rightStick.getButton(9).whenActive(new InstantCommand(m_climber::climberStop, m_climber));
    }


    private void initAutos() {

        auto = new RamseteCommand(trajectory, 
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
                                .andThen(() -> m_drive.stop(), m_drive);
    
        retractHopperCommand = new CmdRetractHopper(m_hopper);
        // climbCommand = new CmdClimb(m_climber);
        
        intakeCargoCommand = new CmdIntakeCargo(m_intake, m_hopper);
        shootCommand = new SequentialCommandGroup(
                          new CmdRetractHopper(m_hopper), 
                          new ParallelCommandGroup(new InstantCommand(m_hopper::runHopper, m_hopper), 
                          new CmdShoot(m_shooter, Shooter.ShooterState.LAUNCHPAD)));

        shootCommand2 = new SequentialCommandGroup(new CmdRetractHopper(m_hopper), 
                          new ParallelCommandGroup(new InstantCommand(m_hopper::runHopper, m_hopper), 
                          new CmdShoot(m_shooter, Shooter.ShooterState.LAUNCHPAD)));
        manualShoot = new CmdShoot(m_shooter, Shooter.ShooterState.LAUNCHPAD);

        // Setup auto-selector
        NarwhalDashboard.addAuto("Basic Auto", auto);
        // NarwhalDashboard.addAuto("Ball Pursuit", cmdBallPursuit);
    }

    private void dashboardInit() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            SmartDashboard.putData("Drivetrain", m_drive);
            SmartDashboard.putData("Intake", m_intake);
            SmartDashboard.putData("Hopper", m_hopper);
            SmartDashboard.putData("Shooter", m_shooter);
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
        NarwhalDashboard.put("range", "");
        
        NarwhalDashboard.put("Hopper encoder val", m_hopper.getEncVal());
    }

    public Command getAutonomousCommand() {
        // TODO: MAKE HASHMAP CONTAINING INITIAL POSE2D AND AUTO
        m_drive.resetPose(trajectory.getInitialPose());
        return NarwhalDashboard.getSelectedAuto();
    }

}
