package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.LimelightSubsystem;

public class CmdShootTurnVision extends SequentialCommandGroup {

    private LimelightSubsystem limelights;

    /**
     * Rotates the robot a given amount of degrees, then interrupts and aligns to hub if limelight sees target
     * @param turnDeg degrees to turn the robot (- = right, + = left), -170 and 170 work well
     * @Requirements Drivetrain
     */
    public CmdShootTurnVision(double turnDeg) {
        limelights = LimelightSubsystem.getInstance();

        addCommands(
            new InstantCommand(limelights::turnShooterLEDOn),
            new CmdInPlaceTurn(turnDeg).withInterrupt(() -> limelights.getShooterHasValidTarget()),
            new CmdShootAlign().withTimeout(3)
        );
    }
}
