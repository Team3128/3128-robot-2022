package frc.team3128.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.commands.CmdAlign;
import frc.team3128.commands.Shoot;
import frc.team3128.common.limelight.Limelight;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.NAR_Drivetrain;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Sidekick;
import frc.team3128.subsystems.Shooter.ShooterState;

public class AutoSimple extends SequentialCommandGroup {
    public AutoSimple(Shooter shooter, Sidekick sidekick, NAR_Drivetrain drive, Limelight shooterLimelight, Intake intake, Command trajectory) {
        addCommands(
            new ParallelCommandGroup(new Shoot(shooter, sidekick, ShooterState.MID_RANGE), new CmdAlign(drive, shooterLimelight)),
            new InstantCommand(() -> intake.runIntake()),
            trajectory,
            new InstantCommand(() -> intake.stopIntake())
        );
    }
}
