package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;

public class CmdExtendIntakeAndRun extends SequentialCommandGroup{

    public CmdExtendIntakeAndRun(Intake intake, Hopper hopper){
        addCommands(
            new CmdExtendIntake(intake).withTimeout(0.125),
            new CmdIntakeCargo(intake, hopper)
        );
    }
}