package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Hood;

public class CmdShoot extends SequentialCommandGroup {

    /**
     * Shoot without aligning, uses the limelight distance 
     * @Requirements Shooter, Hood 
     */
    public CmdShoot() {
        addCommands(
            new ProxyScheduleCommand(new CmdRetractHopper()),
            new CmdShootDist()
        );
    }

    /**
     * Shoot without aligning using parameter RPM and angle
     * @Requirements Shooter, Hood
     */
    public CmdShoot(double RPM, double angle) {
        addCommands(
            new ProxyScheduleCommand(new CmdRetractHopper()),
            parallel (
                new InstantCommand(() -> Hood.getInstance().startPID(angle), Hood.getInstance()),
                new CmdShootRPM(RPM)
            )
        );
    }
    
}
