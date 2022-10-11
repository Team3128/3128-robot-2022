package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Hopper;

public class CmdShoot extends SequentialCommandGroup {

    /**
     * Shoot without aligning, uses the limelight distance 
     * @Requirements Shooter, Hood 
     */
    public CmdShoot() {
        addCommands(
            // run hopper back to push balls against intake
            new CmdRetractHopper(),
            parallel(
                // run hopper back at slower power to keep balls away from shooter until ready
                new CmdHopperShooting(),
                // shoot in parallel
                new CmdShootDist()
            )
        );
    }

    /**
     * Shoot without aligning using parameter RPM and angle
     * @Requirements Shooter, Hood
     */
    public CmdShoot(double RPM, double angle) {
        addCommands(
            // run hopper back to push balls against intake
            new CmdRetractHopper(),
            parallel(
                // run hopper back at slower power to keep balls away from shooter until ready
                new CmdHopperShooting(),
                new InstantCommand(() -> Hood.getInstance().startPID(angle), Hood.getInstance()),
                new CmdShootRPM(RPM)
            )
        );
    }
    
}
