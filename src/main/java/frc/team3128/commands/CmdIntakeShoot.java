package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.Shooter;

public class CmdIntakeShoot extends CommandBase {
    private Hopper m_hopper;
    private Intake m_intake;
    private Shooter m_shooter;
    private Hood m_hood;
    private LimelightSubsystem limelights;

    /**
     * Climbs robot to Traversal rung autonomously using gyro
     * @Requirements Climber
     */
    public CmdIntakeShoot() {
        m_hopper = Hopper.getInstance();
        m_intake = Intake.getInstance();
        m_shooter = Shooter.getInstance();
        m_hood = Hood.getInstance();
        limelights = LimelightSubsystem.getInstance();
        addRequirements(m_hopper, m_intake, m_shooter, m_hood);
    }

    @Override
    public void initialize() {
        m_intake.runIntake();
        m_hopper.runHopper();
        limelights.turnShooterLEDOn();
    }

    @Override
    public void execute() {
        double dist = limelights.calculateShooterDistance();
        m_shooter.beginShoot(m_shooter.calculateRPMFromDist(dist));
        m_hood.startPID(m_hood.calculateAngleFromDist(dist));
    }
    
    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntake();
        m_hopper.stopHopper();
        m_shooter.stopShoot();
        // limelights.turnShooterLEDOff();
    }

}
