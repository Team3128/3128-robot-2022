package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Hood;

public class CmdShoot extends SequentialCommandGroup {

    public CmdShoot() {
        addCommands(
            new CmdRetractHopper(),
            new CmdShootDist()
        );
    }

    public CmdShoot(double RPM, double angle) {
        addCommands(
            new CmdRetractHopper(),
            new ParallelCommandGroup(
                new InstantCommand(() -> Hood.getInstance().startPID(angle), Hood.getInstance()),
                new CmdShootRPM(RPM))
        );
    }

    public CmdShoot(int RPM, int angle) {
        addCommands(
            new CmdRetractHopper(),
            new ParallelCommandGroup(
                new InstantCommand(() -> Hood.getInstance().startPID(angle), Hood.getInstance()),
                new CmdShootRPM(RPM))
        );
    }
    
}
