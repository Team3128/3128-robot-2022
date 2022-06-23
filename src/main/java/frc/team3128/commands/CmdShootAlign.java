package frc.team3128.commands;

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
            new ProxyScheduleCommand(new CmdRetractHopper()),
            new ScheduleCommand(new InstantCommand(() -> Hopper.getInstance().runHopper(-0.1), Hopper.getInstance())),
            parallel (
                new CmdAlign(),
                new CmdShootDist()
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        // could also do .andThen()
        new InstantCommand(() -> Hopper.getInstance().stopHopper(), Hopper.getInstance()).schedule();
    }
    
}
