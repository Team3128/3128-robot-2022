package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Climber;

public class Climb extends SequentialCommandGroup{

    public Climb(Climber m_climber){
        addCommands(
            new InstantCommand(() -> m_climber.climberExtend()),
            new InstantCommand(() -> m_climber.extendArm()),
            new InstantCommand(() -> m_climber.climberExtend()),
            new InstantCommand(() -> m_climber.climberRetract()),
            new InstantCommand(() -> m_climber.retractArm()),
            new InstantCommand(() -> m_climber.climberRetract())
        );
    }

}