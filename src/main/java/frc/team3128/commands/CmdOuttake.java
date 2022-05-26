package frc.team3128.commands;
import frc.team3128.subsystems.Intake;
import static frc.team3128.Constants.IntakeConstants.*;
import static frc.team3128.Constants.HopperConstants.*;
import frc.team3128.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdOuttake extends CommandBase{
    private Intake m_intake; 
    private Hopper m_hopper;
    private double intakePower;
    private double hopperPower;

    public CmdOuttake(){
        this(OUTTAKE_MOTOR_POWER, REVERSE_HOPPER_MOTOR_POWER);
    }

    public CmdOuttake(double power){
        this(-power, -1.0);
    }

    public CmdOuttake(double intakePower, double hopperPower){
        m_intake = Intake.getInstance();
        m_hopper = Hopper.getInstance();
        this.intakePower = intakePower;
        this.hopperPower = hopperPower;

        addRequirements(m_intake, m_hopper);
    }

    @Override
    public void initialize() {
        m_intake.runIntake(intakePower);
        m_hopper.reverseHopper(hopperPower);
    }

    @Override
    public void end(boolean interrupted){
        m_intake.stopIntake();
        m_hopper.stopHopper();
        m_intake.retractIntake();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
