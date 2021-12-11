package frc.team3128;

import frc.team3128.subsystems.*;
import frc.team3128.subsystems.Shooter.ShooterState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team3128.commands.*;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.limelight.LEDMode;
import frc.team3128.common.limelight.Limelight;

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
    private Sidekick m_sidekick;
    private Hopper m_hopper;
    private Intake m_intake;
    private Climber m_climber;

    private Limelight shooterLimelight = new Limelight("limelight-sog", -26.0, 0, 0, 30);

    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();
    private Command auto;

    public RobotContainer() {

        m_drive = NAR_Drivetrain.getInstance();
        m_shooter = Shooter.getInstance();
        m_sidekick = Sidekick.getInstance();
        m_hopper = Hopper.getInstance();
        m_intake = Intake.getInstance();
        m_climber = Climber.getInstance();

        //Enable all PIDSubsystems so that useOutput runs
        m_shooter.enable();
        m_sidekick.enable();

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);

        shooterLimelight.setLEDMode(LEDMode.ON);

        m_commandScheduler.setDefaultCommand(m_drive, new ArcadeDrive(m_drive, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle));
        m_commandScheduler.setDefaultCommand(m_hopper, new HopperDefault(m_hopper, m_shooter::atSetpoint));

        configureButtonBindings();
        dashboardInit();
    }   

    private void configureButtonBindings() {


        // right button trigger: intake
        m_rightStick.getButton(1).whenActive(new RunCommand(m_intake::runIntake, m_intake));

        // right button 2: shoot
        m_rightStick.getButton(2).whenActive(new ParallelCommandGroup(new Shoot(m_shooter, m_sidekick, ShooterState.MID_RANGE), new CmdAlign(m_drive, shooterLimelight)));

        // right button 9: move arm down
        m_rightStick.getButton(9).whenActive(new RunCommand(m_intake::moveArmDown, m_intake));
        
        // right button 10: move arm up
        m_rightStick.getButton(10).whenActive(new RunCommand(m_intake::moveArmUp, m_intake));

        // left button 7: move climber up
        m_leftStick.getButton(7).whenActive(new RunCommand(m_climber::moveClimberUp, m_climber));

        // left button 8: move climber down
        m_leftStick.getButton(8).whenActive(new RunCommand(m_climber::moveClimberDown, m_climber));
        
    }

    private void dashboardInit() {
        SmartDashboard.putData(CommandScheduler.getInstance());
        
        SmartDashboard.putData(m_drive);
        SmartDashboard.putData(m_shooter);
        SmartDashboard.putData(m_sidekick);
        SmartDashboard.putData(m_hopper);
        SmartDashboard.putData(m_intake);
        SmartDashboard.putData(m_climber);

    }

    public void stopDrivetrain() {
        m_drive.stop();
    }

    public Command getAutonomousCommand() {
        return auto;
    }
}
