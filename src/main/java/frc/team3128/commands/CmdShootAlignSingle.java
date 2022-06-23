package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CmdShootAlignSingle extends SequentialCommandGroup {

    /**
     * Align and shoot using the limelight approximated distance 
     * @Requirements Drivetrain, Shooter, Hood
     */
    public CmdShootAlignSingle() {
        addCommands(
            new ProxyScheduleCommand(new CmdRetractHopper()),
            parallel (
                new CmdAlign(),
                new CmdShootSingleBall()
            )
        );
    }
    
}
