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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team3128.commands.ArcadeDrive;
import frc.team3128.commands.CmdBallJoystickPursuit;
import frc.team3128.commands.CmdBallPursuit;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.limelight.Limelight;

import frc.team3128.subsystems.NAR_Drivetrain;

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

    private Limelight ballLimelight;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

    private String trajJson = "paths/jude_path_o_doom.wpilib.json";
    private Trajectory trajectory = new Trajectory();
    private Command auto;

    private boolean DEBUG = false;

    public RobotContainer() {

        m_drive = NAR_Drivetrain.getInstance();

        //Enable all PIDSubsystems so that useOutput runs

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);

        ballLimelight = new Limelight("limelight-sog", Constants.VisionContants.BALL_LL_ANGLE, Constants.VisionContants.BALL_LL_HEIGHT, 0, 0);

        m_commandScheduler.setDefaultCommand(m_drive, new ArcadeDrive(m_drive, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle));

        try {
            Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(trajJson);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajPath);
        } catch (IOException ex) {
            DriverStation.reportError("Me me no open trajectory: " + trajJson, ex.getStackTrace());
        }


        initAutos();
        configureButtonBindings();
        dashboardInit();
    }   

    private void configureButtonBindings() {
        m_rightStick.getButton(1).whenHeld(new CmdBallJoystickPursuit(m_drive, ballLimelight, m_rightStick));
        m_rightStick.getButton(3).whenPressed(new CmdBallPursuit(m_drive, ballLimelight));
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
        m_drive.resetPose(trajectory.getInitialPose()); // change this if the trajectory being run changes
        return auto;
    }
}
