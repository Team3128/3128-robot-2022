package frc.team3128;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.autonomous.Trajectories;
import frc.team3128.commands.ArcadeDrive;
import frc.team3128.commands.Shoot;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.NAR_Drivetrain;
import frc.team3128.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private NAR_Drivetrain m_drive;

    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;
    private Intake m_intake;

    private Shooter shooter;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();
    private Command auto;

    private boolean DEBUG = false;

    public RobotContainer() {

        m_drive = NAR_Drivetrain.getInstance();

        //Enable all PIDSubsystems so that useOutput runs

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);

        m_commandScheduler.setDefaultCommand(m_drive, new ArcadeDrive(m_drive, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle));

        initAutos();
        configureButtonBindings();
        dashboardInit();
    }   

    private void configureButtonBindings() {
        Shoot shootCmd = new Shoot(shooter, Shooter.ShooterState.LAUNCHPAD);
        m_rightStick.getButton(2).whenActive(new SequentialCommandGroup(new PrintCommand("button 2 active"), shootCmd));
        m_rightStick.getButton(2).whenReleased(new RunCommand(shooter::stopShoot, shooter));
      
        m_rightStick.getButton(8).whenActive(new RunIntake(m_intake));
        m_rightStick.getButton(8).whenReleased(new RunCommand(m_intake::stopIntake, m_intake));
    }

    private void initAutos() {
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
