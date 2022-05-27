package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CmdExtendIntakeAndRun extends SequentialCommandGroup{

    /**
     * Command to eject intake and intake balls
     * @Requirements Intake, Hopper
     */
    public CmdExtendIntakeAndRun(){
        addCommands(
            new CmdExtendIntake(),
            new CmdIntakeCargo()
        );
    }
}