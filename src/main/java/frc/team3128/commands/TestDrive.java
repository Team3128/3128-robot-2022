/*package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.TestBenchSubsystem;

public class TestDrive extends CommandBase{
    private TestBenchSubsystem bench;

    public TestDrive(TestBenchSubsystem bench){
        this.bench = bench;
        addRequirements(bench);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        bench.drive();
    }

    @Override
    public void end(boolean interrupted){
        bench.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}*/
