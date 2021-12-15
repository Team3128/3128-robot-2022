package frc.team3128;

import frc.team3128.subsystems.*;
import frc.team3128.subsystems.Shooter.ShooterState;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team3128.autonomous.AutoSimple;
import frc.team3128.autonomous.Trajectories;
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

    private Limelight shooterLimelight = new Limelight("limelight-sog", -26.0, 0, 0, 30);;

    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();
    private Command auto;
    private Command trajectory;

    private Command runIntake, stopIntake;
    private Command alignShoot, stopAlignShoot;
    private Command armDown, armUp, stopArm;
    private Command climberDown, climberUp, stopClimber;

    private boolean driveInverted = false;

    private boolean debug = false;

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

        m_commandScheduler.setDefaultCommand(m_drive, new ArcadeDrive(m_drive, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle, this::getDriveInverted));
        m_commandScheduler.setDefaultCommand(m_hopper, new HopperDefault(m_hopper, m_shooter::atSetpoint, m_sidekick::atSetpoint));

        initAutos();
        configureButtonBindings();
        dashboardInit();
    }   

    private void configureButtonBindings() {


        // right button trigger: intake
        m_rightStick.getButton(1).whenPressed(runIntake)
                                .whenReleased(stopIntake);

        // right button 2: shoot
        m_rightStick.getButton(2).whenPressed(alignShoot)
                                .whenReleased(stopAlignShoot);
        // right button 9: move arm down
        m_rightStick.getButton(9).whenPressed(armDown)
                                .whenReleased(stopArm);
        // right button 10: move arm up
        m_rightStick.getButton(10).whenPressed(armUp)
                                .whenReleased(stopArm);

        // left trigger: reverse drive
        m_leftStick.getButton(1).whenPressed(() -> driveInverted = !driveInverted);

        // left button 7: move climber up
        m_leftStick.getButton(7).whenPressed(climberUp)
                                .whenReleased(stopClimber);

        // left button 8: move climber down
        m_leftStick.getButton(8).whenPressed(climberDown)
                                .whenReleased(stopClimber);

        // left button 9: turn LED on
        m_leftStick.getButton(9).whenPressed(shooterLimelight::turnLEDOn);

        // left button 10: turn LED off
        m_leftStick.getButton(10).whenPressed(shooterLimelight::turnLEDOff);

    }

    private void dashboardInit() {
        SmartDashboard.putData(CommandScheduler.getInstance());
        
        SmartDashboard.putData(m_drive);
        SmartDashboard.putData(m_shooter);
        SmartDashboard.putData(m_sidekick);
        SmartDashboard.putData(m_hopper);
        SmartDashboard.putData(m_intake);
        SmartDashboard.putData(m_climber);

        if(debug) {
            SmartDashboard.putNumber("Left Stick Raw X", m_leftStick.getX());
            SmartDashboard.putNumber("Left Stick Raw Y", m_leftStick.getY());
            SmartDashboard.putNumber("Left Stick Raw Z", m_leftStick.getZ());
            SmartDashboard.putNumber("Left Stick Raw Twist", m_leftStick.getTwist());
            SmartDashboard.putNumber("Left Stick Raw Throttle", m_leftStick.getThrottle());

            SmartDashboard.putNumber("Right Stick Raw X", m_rightStick.getX());
            SmartDashboard.putNumber("Right Stick Raw Y", m_rightStick.getY());
            SmartDashboard.putNumber("Right Stick Raw Z", m_rightStick.getZ());
            SmartDashboard.putNumber("Right Stick Raw Twist", m_rightStick.getTwist());
            SmartDashboard.putNumber("Right Stick Raw Throttle", m_rightStick.getThrottle());
        }

    }

    private void initAutos() {
        runIntake = new RunCommand(m_intake::runIntake, m_intake);
        stopIntake = new RunCommand(m_intake::stopIntake, m_intake);
        
        alignShoot = new ParallelCommandGroup(new CmdShoot(m_shooter, m_sidekick, ShooterState.MID_RANGE), new CmdAlign(m_drive, shooterLimelight));
        stopAlignShoot = new ParallelCommandGroup(new RunCommand(m_shooter::stopShoot, m_shooter), new InstantCommand(shooterLimelight::turnLEDOff));

        armDown = new RunCommand(m_intake::moveArmDown, m_intake);
        armUp = new RunCommand(m_intake::moveArmUp, m_intake);
        stopArm = new RunCommand(m_intake::stopArm, m_intake);

        climberUp = new RunCommand(m_climber::moveClimberUp, m_climber);
        climberDown = new RunCommand(m_climber::moveClimberDown, m_climber);
        stopClimber = new RunCommand(m_climber::stopClimber, m_climber);

        trajectory = new RamseteCommand(Trajectories.trajectorySimple, 
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
                                        .andThen(() -> m_drive.stop(), m_drive)
                                        .andThen(() -> SmartDashboard.putNumber("End auto heading", m_drive.getHeading()));

        auto = new AutoSimple(m_shooter, m_sidekick, m_drive, shooterLimelight, m_intake, trajectory);
    }

    private boolean getDriveInverted() {
        return driveInverted;
    }

    public void stopDrivetrain() {
        m_drive.stop();
    }

    public Command getAutonomousCommand() {
        return auto;
    }


}
