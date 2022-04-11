package frc.team3128.commands;
import frc.team3128.subsystems.Intake;
import frc.team3128.Constants;
import frc.team3128.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdOuttake extends CommandBase{
    private Intake m_intake; 
    private Hopper m_hopper;
    private double intakePower;
    private double hopperPower;

    public CmdOuttake(Intake intake, Hopper hopper){
        m_intake = intake;
        m_hopper = hopper;
        intakePower = Constants.IntakeConstants.OUTTAKE_MOTOR_POWER;
        hopperPower = Constants.HopperConstants.REVERSE_HOPPER_MOTOR_POWER;

        addRequirements(m_intake, m_hopper);
    }

    public CmdOuttake(Intake intake, Hopper hopper, double power){
        m_intake = intake;
        m_hopper = hopper;
        intakePower = power;
        hopperPower = -1.0;

        addRequirements(m_intake, m_hopper);
    }

    public CmdOuttake(Intake intake, Hopper hopper, double intakePower, double hopperPower){
        m_intake = intake;
        m_hopper = hopper;
        this.intakePower = intakePower;
        this.hopperPower = -1*hopperPower;

        addRequirements(m_intake, m_hopper);
    }

    @Override
    public void initialize() {
        m_intake.runIntakeBack(intakePower);
        m_hopper.reverseHopper(hopperPower);
    }

    @Override
    public void end(boolean interrupted){
        m_intake.stopIntake();
        m_hopper.stopHopper();
        m_intake.retractIntake();
    }

    public boolean isFinished(){
        return false;
    }
}
