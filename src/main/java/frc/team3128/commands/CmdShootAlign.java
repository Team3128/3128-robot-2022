package frc.team3128.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Shooter;

public class CmdShootAlign extends ParallelCommandGroup {

    /**
     * Align and shoot using the limelight approximated distance 
     * @Requirements Drivetrain, Shooter, Hood
     */
    public CmdShootAlign() {
        addCommands(
            new CmdAlign(),
            sequence(
                // run hopper back to push balls against intake
                new CmdRetractHopper(),
                parallel(
                    // run hopper back at slower power to keep balls away from shooter until ready
                    new CmdHopperShooting(),
                    // shoot in parallel
                    new CmdShootDist()
                )
            )
        );
    }
    
}
