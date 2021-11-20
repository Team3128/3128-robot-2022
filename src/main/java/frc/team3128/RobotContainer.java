package frc.team3128;

import frc.team3128.subsystems.*;
import frc.team3128.subsystems.Shooter.ShooterState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3128.commands.*;
import frc.team3128.hardware.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Mover m_move; // test subsystem
    private NAR_Drivetrain m_drive;
    private Shooter m_shooter;

    private NAR_Joystick m_stick;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();
    private Command auto;

    public RobotContainer() {
        m_move = new Mover();
        m_drive = new NAR_Drivetrain();
        m_stick = new NAR_Joystick(0);
        m_shooter = new Shooter();

        m_commandScheduler.registerSubsystem(m_move, m_drive, m_shooter);

        m_commandScheduler.setDefaultCommand(m_drive, new ArcadeDrive(m_drive, m_stick::getY, m_stick::getX));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        m_stick.getButton(1).whenPressed(new Shoot(m_shooter, ShooterState.MID_RANGE))
                            .whenReleased(new StopShoot(m_shooter));
    }

    public void stopDrivetrain() {
        m_drive.stop();
    }

    public Command getAutonomousCommand() {
        return auto;
    }
}
