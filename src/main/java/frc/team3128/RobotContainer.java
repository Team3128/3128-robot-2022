package frc.team3128;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team3128.commands.ArcadeDrive;
//import frc.team3128.commands.TestDrive;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.NAR_Drivetrain;
import frc.team3128.subsystems.TestBenchPiston;
import frc.team3128.subsystems.TestBenchMotor;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    //private NAR_Drivetrain m_drive;
    private TestBenchMotor testBenchMotor;
    private TestBenchPiston testBenchPiston;
    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;
    private Limelight lime;
    
    private Command auto;

    private Thread dashboardUpdateThread;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

    private boolean DEBUG = false;

    public RobotContainer() {

        //m_drive = NAR_Drivetrain.getInstance();

        //Enable all PIDSubsystems so that useOutput runs

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);
        testBenchPiston = new TestBenchPiston();
        testBenchMotor = new TestBenchMotor(); 
        //m_commandScheduler.setDefaultCommand(testBenchSubsystem, new TestDrive(testBenchSubsystem));

        lime = new Limelight("limelight-bog", 0, 0, 0, 0);

        configureButtonBindings();
        dashboardInit();
        initAutos();
    }   

    private void initAutos() {
        Command auto1 = new InstantCommand();
        Command auto2 = new InstantCommand();

        // Setup auto chooser
        NarwhalDashboard.clearAutos();
        NarwhalDashboard.addAuto("Auto test 1", auto1);
        NarwhalDashboard.addAuto("Auto test 2", auto2);
        NarwhalDashboard.pushData();
    }

    private void configureButtonBindings() {
      
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

        // Setup auto-selector
        NarwhalDashboard.addAuto("Auto test", auto);
        // NarwhalDashboard.addAuto("Ball Pursuit", cmdBallPursuit);
    }

    private void dashboardInit() {
        if (DEBUG) {
            //SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            //SmartDashboard.putData("Drivetrain", m_drive);
        }
        NarwhalDashboard.startServer();
        setupLimelights(lime);           
    }

    public void stopDrivetrain() {
        //m_drive.stop();
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
        NarwhalDashboard.put("rpm", testBenchMotor.getRPM());
        NarwhalDashboard.put("range", ""); // fix this
    }
}
