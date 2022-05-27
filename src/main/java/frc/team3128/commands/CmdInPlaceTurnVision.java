package frc.team3128.commands;

import frc.team3128.subsystems.LimelightSubsystem;

public class CmdInPlaceTurnVision extends CmdInPlaceTurn {

    private LimelightSubsystem limelights;

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
