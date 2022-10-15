package frc.team3128;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.team3128.Constants.HoodConstants.*;

import static frc.team3128.Constants.ClimberConstants.*;

import frc.team3128.commands.CmdAlign;
import frc.team3128.commands.CmdArcadeDrive;
import frc.team3128.commands.CmdBallJoystickPursuit;
import frc.team3128.commands.CmdClimbEncoder;
import frc.team3128.commands.CmdClimbTraversalGyro;
import frc.team3128.commands.CmdExtendIntake;
import frc.team3128.commands.CmdExtendIntakeAndRun;
import frc.team3128.commands.CmdHopperShooting;
import frc.team3128.commands.CmdIntakeCargo;
import frc.team3128.commands.CmdOuttake;
import frc.team3128.commands.CmdRetractHopper;
import frc.team3128.commands.CmdShoot;
import frc.team3128.commands.CmdShootAlign;
import frc.team3128.commands.CmdShootDist;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.hardware.limelight.LEDMode;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.LimelightSubsystem;
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
    private Shooter m_shooter;
    private Intake m_intake;   
    private Hopper m_hopper;
    private Climber m_climber;
    private Hood m_hood;
    private LimelightSubsystem m_ll;

    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;
    private NAR_XboxController m_operatorController;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();
  
    private boolean DEBUG = true; 

    public RobotContainer() {
        // ConstantsInt.initTempConstants();
        m_drive = NAR_Drivetrain.getInstance();
        m_shooter = Shooter.getInstance();
        m_intake = Intake.getInstance();
        m_hopper = Hopper.getInstance();
        m_climber = Climber.getInstance();
        m_hood = Hood.getInstance();
        m_ll = LimelightSubsystem.getInstance();

        //Enable all PIDSubsystems so that useOutput runs
        m_shooter.enable();
        m_hood.enable();

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);
        m_operatorController = new NAR_XboxController(2);

        m_commandScheduler.setDefaultCommand(m_drive, new CmdArcadeDrive(m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle));

        initDashboard();
        configureButtonBindings();
        // configureDriverOperator();
        
        if(RobotBase.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {

        // RIGHT
        // m_rightStick.getButton(1).whenPressed(m_shooter::runShooter).whenReleased(m_shooter::stopShoot);
        m_rightStick.getButton(1).whenHeld(new CmdShootAlign());

        // When interpolating, uncomment this and the lines in Shooter.java and Hood.java calling ConstantsInt
        // m_rightStick.getButton(1).whenHeld(new CmdShoot(2700, 12));

        m_rightStick.getButton(2).whenHeld(new CmdExtendIntakeAndRun())
                                .whenReleased(new CmdIntakeCargo().withTimeout(0.25));
        
        // m_rightStick.getButton(3).whenHeld(new ParallelCommandGroup(
        //             new CmdBallJoystickPursuit(
        //                 m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle),
        //             new CmdExtendIntakeAndRun()) 

        // lower hub shot
        m_rightStick.getButton(3).whenHeld(
                    new ParallelCommandGroup(
                        new CmdShoot(1200, 34.4),
                        new RunCommand(m_drive::stop, m_drive)));

        // ram shot
        m_rightStick.getButton(4).whenHeld(
                        new CmdShoot(2800, 13.4));

        m_rightStick.getButton(5).whenPressed(new CmdClimbTraversalGyro());

        m_rightStick.getButton(7).whenPressed(new CmdClimbEncoder(0));

        m_rightStick.getButton(8).whenHeld(new SequentialCommandGroup(
                                                new CmdExtendIntake().withTimeout(0.1), 
                                                new CmdOuttake()));

        m_rightStick.getButton(10).whenPressed(new InstantCommand(m_climber::bothStop, m_climber));

        m_rightStick.getButton(11).whenPressed(new CmdExtendIntake());

        m_rightStick.getButton(13).whenPressed(() -> m_hood.startPID(MIN_ANGLE));

        m_rightStick.getButton(14).whenPressed(() -> m_hood.startPID(MAX_ANGLE));

        m_rightStick.getButton(16).whenPressed(() -> m_intake.retractIntake());

        m_rightStick.getUpPOVButton().whenPressed(() -> m_ll.turnShooterLEDOn());
        m_rightStick.getDownPOVButton().whenPressed(() -> m_ll.turnShooterLEDOff());
        

        // LEFT

        m_leftStick.getButton(2).whenPressed(() -> m_climber.resetLeftEncoder());        

        m_leftStick.getButton(5).whenPressed(() -> m_hood.zeroEncoder()); 

        m_leftStick.getButton(8).whenPressed(new CmdClimbEncoder(CLIMB_ENC_TO_TOP));

        m_leftStick.getUpPOVButton().whenPressed(new InstantCommand(m_climber::bothExtend, m_climber))
                                    .whenReleased(new InstantCommand(m_climber::bothStop, m_climber));

        m_leftStick.getDownPOVButton().whenPressed(new InstantCommand(m_climber::bothRetract, m_climber))
                                    .whenReleased(new InstantCommand(m_climber::bothStop, m_climber));
        
        m_leftStick.getRightPOVButton().whenPressed(new InstantCommand(m_climber::extendPiston, m_climber));
        m_leftStick.getLeftPOVButton().whenPressed(new InstantCommand(m_climber::retractPiston, m_climber));

        m_leftStick.getButton(11).whenPressed(new InstantCommand(m_climber::bothManualExtend, m_climber))
                                .whenReleased(new InstantCommand(m_climber::bothStop, m_climber));

        m_leftStick.getButton(16).whenPressed(new InstantCommand(m_climber::bothManualRetract, m_climber))
                                .whenReleased(new InstantCommand(m_climber::bothStop, m_climber));

        m_leftStick.getButton(13).whenPressed(new InstantCommand(m_climber::bothExtend, m_climber))
                                .whenReleased(new InstantCommand(m_climber::bothStop, m_climber));

        m_leftStick.getButton(14).whenPressed(new InstantCommand(m_climber::bothRetract, m_climber))
                                .whenReleased(new InstantCommand(m_climber::bothStop, m_climber));

        m_leftStick.getButton(12).whenPressed(new InstantCommand(m_climber::extendPiston, m_climber));
        m_leftStick.getButton(15).whenPressed(new InstantCommand(m_climber::retractPiston, m_climber));
    }

    public void configureDriverOperator() {
        m_operatorController.getRightTrigger().whileActiveOnce(new CmdShootAlign());

        m_operatorController.getLeftTrigger().whileActiveOnce(
            new CmdShoot(2800, 13.4));

        // When interpolating, uncomment this and the lines in Shooter.java and Hood.java calling ConstantsInt
        // m_operatorController.getRightTrigger().whileActiveOnce(new CmdShoot(2700, 12));

        m_operatorController.getButton("RightBumper").whileActiveOnce(new CmdExtendIntakeAndRun())
                                            .whenInactive(new CmdIntakeCargo().withTimeout(0.25));
                                    
        m_operatorController.getButton("LeftBumper").whileActiveOnce(new SequentialCommandGroup(
                                            new CmdExtendIntake().withTimeout(0.1), 
                                            new CmdOuttake()));

        m_operatorController.getButton("RightStick").whenActive(new CmdClimbEncoder(CLIMB_ENC_TO_TOP));

        m_operatorController.getButton("LeftStick").whileActiveOnce(
            new ParallelCommandGroup(
                new CmdShoot(1200, 34.4),
                new RunCommand(m_drive::stop, m_drive)));

        m_operatorController.getButton("Start").whenActive(new CmdClimbTraversalGyro());
        m_operatorController.getButton("Back").whenActive(new InstantCommand(m_climber::bothStop, m_climber));

        // m_operatorController.getButton("Start").whenActive(new InstantCommand(m_climber::bothManualExtend, m_climber))
        // .whenReleased(new InstantCommand(m_climber::bothStop, m_climber));

        // m_operatorController.getButton("Back").whenActive(new InstantCommand(m_climber::bothManualRetract, m_climber))
        // .whenReleased(new InstantCommand(m_climber::bothStop, m_climber));

        m_operatorController.getButton("B").whenActive(new InstantCommand(m_climber::extendPiston, m_climber));

        m_operatorController.getButton("X").whenActive(new InstantCommand(m_climber::retractPiston, m_climber));

        m_operatorController.getButton("Y").whenActive(new InstantCommand(m_climber::bothExtend, m_climber))
        .whenInactive(new InstantCommand(m_climber::bothStop, m_climber));

        m_operatorController.getButton("A").whenActive(new InstantCommand(m_climber::bothRetract, m_climber))
        .whenInactive(new InstantCommand(m_climber::bothStop, m_climber));

        // RIGHT 

        m_rightStick.getButton(5).whenPressed(() -> m_hood.zeroEncoder()); 

        m_rightStick.getButton(6).whenPressed(() -> m_climber.resetLeftEncoder());

        // m_rightStick.getButton(13).whenPressed(() -> m_hood.startPID(MIN_ANGLE));

        // m_rightStick.getButton(14).whenPressed(() -> m_hood.startPID(MAX_ANGLE));
        m_rightStick.getButton(14).whenPressed(() -> m_shooter.reverseShoot());

        m_rightStick.getUpPOVButton().whenPressed(() -> m_ll.turnShooterLEDOn());
        m_rightStick.getDownPOVButton().whenPressed(() -> m_ll.turnShooterLEDOff());
    }

    public void init() {
        m_climber.retractPiston();
        m_intake.retractIntake();
        m_hood.startPID(HOME_ANGLE);
    }

    private void initDashboard() {
        NAR_Shuffleboard.addComplex("General","Drivetrain",m_drive).withSize(3, 1).withPosition(0,0);
        NAR_Shuffleboard.addComplex("General","Intake",m_intake).withSize(3, 1).withPosition(3,0);
        NAR_Shuffleboard.addComplex("General","Hopper",m_hopper).withSize(3, 1).withPosition(6,0);
        NAR_Shuffleboard.addComplex("General","Climber",m_climber).withSize(3, 1).withPosition(0,1);
        NAR_Shuffleboard.addComplex("General","Shooter",m_shooter).withSize(3, 1).withPosition(3, 1);
        NAR_Shuffleboard.addComplex("General","Hood",m_hood).withSize(3, 1).withPosition(6,1);

        m_drive.initShuffleboard();
        m_intake.initShuffleboard();
        m_hopper.initShuffleboard();
        m_climber.initShuffleboard();
        m_shooter.initShuffleboard();
        m_hood.initShuffleboard();
        m_ll.initShuffleboard();

        NarwhalDashboard.setSelectedLimelight(m_ll.getBallLimelight());
        NarwhalDashboard.startServer();   
        
        Log.info("NarwhalRobot", "Setting up limelight chooser...");
      
        for (Limelight ll : new Limelight[] {m_ll.getShooterLimelight(), m_ll.getBallLimelight()}) {
            NarwhalDashboard.addLimelight(ll);
            ll.setLEDMode(LEDMode.OFF);
        }
    }

    public void updateDashboard() {
        NAR_Shuffleboard.update();
        NarwhalDashboard.put("time", Timer.getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("rpm", m_shooter.getMeasurement());
        NarwhalDashboard.put("range", m_ll.calculateShooterDistance());
        NarwhalDashboard.put("x", m_drive.getPose().getX());
        NarwhalDashboard.put("y", m_drive.getPose().getY());
        NarwhalDashboard.put("theta", Units.degreesToRadians(m_drive.getHeading()));
        NarwhalDashboard.put("climbEnc", m_climber.getCurrentTicks());
    }
}
