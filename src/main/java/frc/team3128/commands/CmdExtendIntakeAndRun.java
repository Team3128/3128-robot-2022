package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CmdExtendIntakeAndRun extends SequentialCommandGroup{

    public CmdExtendIntakeAndRun(){
        addCommands(
            new CmdExtendIntake().withTimeout(0.125),
            new CmdIntakeCargo()
        );
    }
}