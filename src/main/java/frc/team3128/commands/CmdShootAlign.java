package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CmdShootAlign extends SequentialCommandGroup {

    public CmdShootAlign() {
        addCommands(
            new CmdRetractHopper(),
            new ParallelCommandGroup(
                new CmdAlign(),
                new CmdShootDist()
            )
        );
    }
    
}
