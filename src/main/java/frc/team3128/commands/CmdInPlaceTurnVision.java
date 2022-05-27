package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import static frc.team3128.Constants.DriveConstants.*;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdInPlaceTurnVision extends CmdInPlaceTurn {

    private NAR_Drivetrain drivetrain;
    private LimelightSubsystem limelights;
    private double turnDeg;

    /**
     * Rotates the robot a given amount of degrees and interrupts when limelight sees hub target
     * @param turnDeg degrees to turn the robot
     * @Requirements Drivetrain
     */
    public CmdInPlaceTurnVision(double turnDeg) {
        super(turnDeg);
        limelights = LimelightSubsystem.getInstance();
    }

    @Override
    public void initialize() {
        super.initialize();
        limelights.turnShooterLEDOn();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || limelights.getShooterHasValidTarget();
    }

    @Override
    public void end(boolean interrupted) {
        limelights.turnShooterLEDOff();
    }

}
