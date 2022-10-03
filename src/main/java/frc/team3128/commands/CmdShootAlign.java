package frc.team3128.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Hopper;

public class CmdShootAlign extends SequentialCommandGroup {

    /**
     * Align and shoot using the limelight approximated distance 
     * @Requirements Drivetrain, Shooter, Hood
     */
    public CmdShootAlign() {
        addCommands(
            // run hopper back to push balls against intake
            new ProxyScheduleCommand(new CmdRetractHopper()),
            // run hopper back at slower power to keep balls away from shooter until ready
            new ScheduleCommand(new InstantCommand(() -> Hopper.getInstance().runHopper(-0.1), Hopper.getInstance())),
            // align and shoot in parallel
            parallel (
                new CmdAlign(),
                new CmdShootDist()
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        Hopper.getInstance().stopHopper();
    }
    
}
