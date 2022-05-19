package frc.team3128;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.team3128.Constants.HoodConstants.*;
import static frc.team3128.Constants.ClimberConstants.*;

import frc.team3128.autonomous.AutoPrograms;
import frc.team3128.commands.CmdAlign;
import frc.team3128.commands.CmdArcadeDrive;
import frc.team3128.commands.CmdClimbEncoder;
import frc.team3128.commands.CmdClimbTraversalGyro;
import frc.team3128.commands.CmdExtendIntake;
import frc.team3128.commands.CmdExtendIntakeAndRun;
import frc.team3128.commands.CmdIntakeCargo;
import frc.team3128.commands.CmdOuttake;
import frc.team3128.commands.CmdRetractHopper;
import frc.team3128.commands.CmdShootDist;
import frc.team3128.commands.CmdShootRPM;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.limelight.LEDMode;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Shooter.ShooterState;

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

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

    private AutoPrograms autos;
  
    private boolean DEBUG = true;
    private boolean driveHalfSpeed = false;

    private Trigger isShooting;

    public RobotContainer() {
        ConstantsInt.initTempConstants();
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

        isShooting = new Trigger(m_shooter::isReady);

        m_commandScheduler.setDefaultCommand(m_drive, new CmdArcadeDrive(m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle, () -> driveHalfSpeed));
        //m_commandScheduler.setDefaultCommand(m_hopper, new CmdHopperDefault(m_hopper, m_shooter::isReady)); //TODO: make input into this good method ???

        autos = new AutoPrograms();
        initDashboard();
        initLimelights(m_ll.getShooterLimelight(), m_ll.getBallLimelight()); 
        configureButtonBindings();
        
        if(RobotBase.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {

        //RIGHT
        m_rightStick.getButton(1).whenPressed(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> m_ll.turnShooterLEDOn()),
                        new CmdRetractHopper().withTimeout(0.5), 
                        new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
                        // new CmdExtendIntake(),
                        new ParallelCommandGroup(
                            // new RunCommand(m_intake::runIntake, m_intake),
                            new CmdAlign(), 
                            new InstantCommand(() -> m_hopper.runHopper(-0.1), m_hopper),
                            new CmdShootDist())))
                        .whenReleased(new ParallelCommandGroup(
                            new InstantCommand(m_shooter::stopShoot, m_shooter),
                            new InstantCommand(() -> m_ll.turnShooterLEDOff())));

        // When interpolating, uncomment this and the lines in Shooter.java and Hood.java calling ConstantsInt
        // m_rightStick.getButton(1).whenPressed(
        //             new SequentialCommandGroup(
        //                 new CmdRetractHopper().withTimeout(0.5), 
        //                 new ParallelCommandGroup(
        //                     new InstantCommand(() -> m_hood.startPID(12)), 
        //                     new CmdShootRPM(2700), 
        //                     new InstantCommand(() -> m_hopper.runHopper(-0.1), m_hopper)
        //                 .whenReleased(new ParallelCommandGroup(
        //                     new InstantCommand(m_shooter::stopShoot, m_shooter)));

        m_rightStick.getButton(2).whenHeld(new CmdExtendIntakeAndRun())
                                .whenReleased(new CmdIntakeCargo().withTimeout(0.25));
        
        // m_rightStick.getButton(3).whenHeld(new ParallelCommandGroup(
        //                                     new CmdBallJoystickPursuit(m_drive, m_ballLimelight, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle),
        //                                     new CmdExtendIntakeAndRun(m_intake, m_hopper)).beforeStarting(new WaitCommand(0.5)) // Wait 0.5s, then extend intake so as to not block vision
        //                                 );

        // lower hub shot
        m_rightStick.getButton(3).whenHeld(new SequentialCommandGroup(
                    new CmdRetractHopper().withTimeout(0.5),
                    new InstantCommand(() -> m_shooter.setState(ShooterState.LOWERHUB)),
                    new ParallelCommandGroup(
                        new RunCommand(m_drive::stop, m_drive),
                        new InstantCommand(() -> m_hopper.runHopper(-0.1), m_hopper),
                        new InstantCommand(() -> m_hood.startPID(28)),
                        new CmdShootRPM(1200))))
                    .whenReleased(new ParallelCommandGroup(
                        new InstantCommand(m_shooter::stopShoot, m_shooter),
                        new InstantCommand(() -> m_ll.turnShooterLEDOff())));

        m_rightStick.getButton(4).whenPressed(new SequentialCommandGroup(
                                                new CmdRetractHopper().withTimeout(0.5), 
                                                new ParallelCommandGroup(
                                                        new InstantCommand(() -> m_hood.startPID(7)),
                                                        new CmdShootRPM(2800), 
                                                        new InstantCommand(() -> m_hopper.runHopper(-0.1), m_hopper))))
                                    .whenReleased(new ParallelCommandGroup(new InstantCommand(m_shooter::stopShoot, m_shooter)));

        m_rightStick.getButton(5).whenPressed(new CmdClimbTraversalGyro());
      
        m_rightStick.getButton(6).whenPressed(new CmdClimbEncoder(CLIMB_ENC_TO_TOP));

        m_rightStick.getButton(7).whenPressed(new CmdClimbEncoder(0));

        // extend intake and outtake
        m_rightStick.getButton(8).whenHeld(new SequentialCommandGroup(
                                                new CmdExtendIntake().withTimeout(0.1), 
                                                new CmdOuttake()));

        m_rightStick.getButton(10).whenPressed(new InstantCommand(m_climber::bothStop, m_climber));

        m_rightStick.getButton(11).whenPressed(new CmdExtendIntake());

        m_rightStick.getButton(13).whenPressed(() -> m_hood.startPID(MIN_ANGLE));

        m_rightStick.getButton(14).whenPressed(() -> m_hood.startPID(MAX_ANGLE));

        m_rightStick.getButton(16).whenPressed(() -> m_intake.retractIntake());

        m_rightStick.getPOVButton(0).whenPressed(() -> m_ll.turnShooterLEDOn());
        m_rightStick.getPOVButton(4).whenPressed(() -> m_ll.turnShooterLEDOff());

        //LEFT

        m_leftStick.getButton(1).whenPressed(new SequentialCommandGroup(
                            new CmdRetractHopper().withTimeout(0.5), 
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> m_hood.startPID(ConstantsInt.ShooterConstants.SET_ANGLE)),
                                    new CmdShootRPM(2800), 
                                    new InstantCommand(() -> m_hopper.runHopper(-0.1), m_hopper))))
                .whenReleased(new ParallelCommandGroup(new InstantCommand(m_shooter::stopShoot, m_shooter)));

        m_leftStick.getButton(2).whenPressed(new InstantCommand(m_climber::resetLeftEncoder, m_climber));        

        m_leftStick.getButton(3).whenPressed(() -> driveHalfSpeed = !driveHalfSpeed);

        // m_leftStick.getButton(5).whenPressed(new CmdClimbEncoder(m_climber, -m_climber.getDesiredTicks(SMALL_VERTICAL_DISTANCE)));

        m_leftStick.getButton(5).whenPressed(() -> m_hood.zeroEncoder()); 

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

        m_leftStick.getButton(9).whenPressed(new CmdClimbEncoder(CLIMB_ENC_DIAG_EXTENSION));
        m_leftStick.getButton(8).whenPressed(new CmdClimbEncoder(CLIMB_ENC_TO_TOP));
        m_leftStick.getButton(10).whenPressed(new CmdClimbEncoder(-120));

        // Triggers

        isShooting.debounce(0.1).whenActive(new InstantCommand(m_hopper::runHopper, m_hopper))
                                        .whenInactive(new InstantCommand(m_hopper::stopHopper, m_hopper));
        
    }

    public void init() {
        initPneumatics();
        m_hood.zero();
    }

    private void initDashboard() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            SmartDashboard.putData("Drivetrain", m_drive);
            SmartDashboard.putData("Intake", m_intake);
            SmartDashboard.putData("Hopper", m_hopper);
            SmartDashboard.putData("Climber", m_climber);
            SmartDashboard.putData("Shooter", (PIDSubsystem)m_shooter);
            SmartDashboard.putData("Hood", (PIDSubsystem)m_hood);
        }

        NarwhalDashboard.setSelectedLimelight(m_ll.getBallLimelight());
        NarwhalDashboard.startServer();       
    }

    public void stopDrivetrain() {
        m_drive.stop();
    }
  
    private void initLimelights(Limelight... limelightList) {
        Log.info("NarwhalRobot", "Setting up limelight chooser...");
      
        for (Limelight ll : limelightList) {
            NarwhalDashboard.addLimelight(ll);
            ll.setLEDMode(LEDMode.OFF);
        }

    }

    public void updateDashboard() {

        // Update necessary Nardash debug data

        NarwhalDashboard.put("time", Timer.getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("rpm", m_shooter.getMeasurement());
        NarwhalDashboard.put("range", m_ll.calculateDistance("shooter"));
        NarwhalDashboard.put("x", m_drive.getPose().getX());
        NarwhalDashboard.put("y", m_drive.getPose().getY());
        NarwhalDashboard.put("theta", Units.degreesToRadians(m_drive.getHeading()));
        NarwhalDashboard.put("climbEnc", m_climber.getCurrentTicksLeft());

        // Post miscellaneous other debug data to Smartdash

        SmartDashboard.putNumber("range", m_ll.calculateDistance("shooter"));
        SmartDashboard.putNumber("ty", m_ll.getShooterLimelight().getValue(LimelightKey.VERTICAL_OFFSET, 3));
        SmartDashboard.putNumber("adjusted ty", m_ll.getShooterLimelight().getValue(LimelightKey.VERTICAL_OFFSET, 5) * (2/3));

        // SmartDashboard.putBoolean("Shooter is ready", m_shooter.isReady());
        SmartDashboard.putString("Shooter state", m_shooter.getState().toString());
        SmartDashboard.putNumber("Shooter Setpoint", m_shooter.getSetpoint());
        SmartDashboard.putNumber("Shooter RPM", m_shooter.getMeasurement());

        SmartDashboard.putNumber("Hood Setpoint", m_hood.getSetpoint());
        SmartDashboard.putNumber("Hood angle", m_hood.getMeasurement());

        SmartDashboard.putString("Intake state:", m_intake.getSolenoid());
    }

    public void initPneumatics() {
        m_climber.retractPiston();
        m_intake.retractIntake();
    }

    public Command getAutonomousCommand() {
        return autos.getAutonomousCommand();
    }
}
