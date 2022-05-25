package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import static frc.team3128.Constants.DriveConstants.*;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdInPlaceTurnVision extends PIDCommand {

    private NAR_Drivetrain drivetrain;
    private LimelightSubsystem limelights;
    private double turnDeg;

    public CmdInPlaceTurnVision(double turnDeg) {

        super(
            new PIDController(TURN_kP, TURN_kI, TURN_kD),
            NAR_Drivetrain.getInstance()::getHeading,
            MathUtil.inputModulus(NAR_Drivetrain.getInstance().getHeading() + turnDeg, -180, 180),
            output -> NAR_Drivetrain.getInstance().tankDrive(output + Math.copySign(TURN_kF, output), -output - Math.copySign(TURN_kF, output)),
            NAR_Drivetrain.getInstance()
        );

        this.drivetrain = NAR_Drivetrain.getInstance();
        this.limelights = LimelightSubsystem.getInstance();
        this.turnDeg = turnDeg;

        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(TURN_TOLERANCE);
    }

    @Override
    public void initialize() {
        super.initialize();

        // Hack to make sure the robot turns 180 degrees from current heading and not 180 degrees from 0
        double setpoint = MathUtil.inputModulus(drivetrain.getHeading() + turnDeg, -180, 180);
        m_setpoint = () ->  setpoint;
        limelights.turnShooterLEDOn();
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint() || limelights.getShooterLimelight().hasValidTarget();
    }

    @Override
    public void end(boolean interrupted) {
        limelights.turnShooterLEDOff();
    }

}
