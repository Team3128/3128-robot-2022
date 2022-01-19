package frc.team3128;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.autonomous.Trajectories;
import frc.team3128.commands.ArcadeDrive;
import frc.team3128.commands.Climb;
import frc.team3128.commands.HopperDefault;
import frc.team3128.commands.IntakeCargo;
import frc.team3128.commands.Shoot;
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

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();
    private Command auto;

    private boolean DEBUG = false;

    public RobotContainer() {

        m_drive = NAR_Drivetrain.getInstance();
        m_intake = Intake.getInstance();
        m_shooter = Shooter.getInstance();
        m_hopper = Hopper.getInstance();
        m_climber = Climber.getInstance();

        //Enable all PIDSubsystems so that useOutput runs
        m_shooter.enable();

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);

        m_commandScheduler.setDefaultCommand(m_drive, new ArcadeDrive(m_drive, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle));
        m_commandScheduler.setDefaultCommand(m_hopper, new HopperDefault(m_hopper, m_shooter::atSetpoint)); //TODO: make input into this good method


        initAutos();
        configureButtonBindings();
        dashboardInit();
    }   

    private void configureButtonBindings() {
        // Buttons...
        // right:
        // 1 (trigger): intake 
        // 2: shoot
        // 8: climb
        //
        // left:
        Shoot shootCmd = new Shoot(m_shooter, Shooter.ShooterState.LAUNCHPAD);

        m_rightStick.getButton(1).whenActive(new IntakeCargo(m_intake, m_hopper));
        m_rightStick.getButton(1).whenReleased(new InstantCommand(m_intake::stopIntake, m_intake));

        m_rightStick.getButton(2).whenActive(new SequentialCommandGroup(new PrintCommand("button 2 active"), shootCmd));
        m_rightStick.getButton(2).whenReleased(new InstantCommand(m_shooter::stopShoot, m_shooter));

        m_rightStick.getButton(8).whenActive(new Climb(m_climber));
        m_rightStick.getButton(8).whenReleased(new InstantCommand(m_climber::climbEnd, m_climber));
    }

    private void initAutos() {

        NarwhalDashboard.addAuto("Shoot Launchpad", new Shoot(m_shooter, Shooter.ShooterState.LAUNCHPAD));

        auto = new RamseteCommand(Trajectories.trajectorySimple, 
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
    }

    private void dashboardInit() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            SmartDashboard.putData("Drivetrain", m_drive);
        }
            
    }

    public void stopDrivetrain() {
        m_drive.stop();
    }

    public Command getAutonomousCommand() {
        m_drive.resetPose(Trajectories.trajectorySimple.getInitialPose()); // change this if the trajectory being run changes
        return auto;
    }

}
