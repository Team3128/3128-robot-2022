package frc.team3128;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    private Intake m_intake;
    private Shooter m_shooter;
    private Hopper m_hopper;
    private Climber m_climber;

    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;

    private Limelight m_shooterLimelight;
    private Limelight m_balLimelight;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

    private String trajJson = "paths/jude_path_o_doom.wpilib.json";
    private Trajectory trajectory = new Trajectory();

    private Command auto;
    private IntakeCargo intakeCargoCommand;
    private SequentialCommandGroup shootCommand;
    private Climb climbCommand;

    private boolean DEBUG = false;

    public RobotContainer() {

        m_drive = NAR_Drivetrain.getInstance();

        //Enable all PIDSubsystems so that useOutput runs

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);

        // m_shooterLimelight = new Limelight(hostname, cameraAngle, cameraHeight, frontDistance, targetWidth);
        // m_balLimelight = new Limelight(hostname, cameraAngle, cameraHeight, frontDistance, targetWidth);

        m_commandScheduler.setDefaultCommand(m_drive, new ArcadeDrive(m_drive, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle));

        configureButtonBindings();
        dashboardInit();
        initAutos();
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
        m_rightStick.getButton(1).whenHeld(intakeCargoCommand);

        // m_rightStick.getButton(2).whenActive(new SequentialCommandGroup(new PrintCommand("button 2 active"), shootCommand));
        // m_rightStick.getButton(2).whenReleased(new InstantCommand(m_shooter::stopShoot, m_shooter));
        m_rightStick.getButton(2).whenHeld(shootCommand);

        //m_rightStick.getButton(8).whenActive(climbCommand);
        //m_rightStick.getButton(9).whenActive(new InstantCommand(m_climber::climberStop, m_climber));
        //m_rightStick.getButton(8).whenReleased(new InstantCommand(m_climber::climberStop, m_climber));

        // m_rightStick.getButton(1).whenHeld(new IntakeCargo(m_intake, m_hopper));
        // m_rightStick.getButton(2).whenHeld(new SequentialCommandGroup(new PrintCommand("button 2 active"), shootCmd));
    }
  
    private void initAutos() {

        // Setup auto-selector
        NarwhalDashboard.addAuto("Auto test", auto);
        // NarwhalDashboard.addAuto("Ball Pursuit", cmdBallPursuit);
        
        intakeCargoCommand = new IntakeCargo(m_intake, m_hopper);
        // shootCommand = new SequentialCommandGroup(
        //                new RetractHopper(m_hopper), 
        //                new ParallelCommandGroup(new InstantCommand(m_hopper::runHopper, m_hopper), 
        //                new Shoot(m_shooter, Shooter.ShooterState.LAUNCHPAD)));
        //shootCommand = new Shoot(m_shooter, Shooter.ShooterState.LAUNCHPAD);
        climbCommand = new Climb(m_climber);
    }

    private void dashboardInit() {
        if (DEBUG) {
            //SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            //SmartDashboard.putData("Drivetrain", m_drive);
        }
        NarwhalDashboard.startServer();
        setupLimelights(m_shooterLimelight,m_balLimelight);           
    }

    public void stopDrivetrain() {
        m_drive.stop();
    }

    public Command getAutonomousCommand() {
        return NarwhalDashboard.getSelectedAuto();
    }

    private void setupLimelights(Limelight... limelightList) {
        Log.info("NarwhalRobot", "Setting Up Limelight Chooser...");

        for(Limelight lime : limelightList)
            NarwhalDashboard.addLimelight(lime);
    }

    public void updateDashboard(){
        NarwhalDashboard.put("time", Timer.getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("rpm", m_shooter.getMeasurement());
        NarwhalDashboard.put("range", ""); // fix this
    }
}
