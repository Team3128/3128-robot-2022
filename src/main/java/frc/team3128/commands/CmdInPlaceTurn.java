package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import static frc.team3128.Constants.DriveConstants.*;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdInPlaceTurn extends PIDCommand {

    private NAR_Drivetrain drivetrain;
    private double turnDeg;

    /**
     * Rotates the robot a given amount of degrees
     * @param turnDeg degrees to turn the robot
     * @Requirements Drivetrain
     */
    public CmdInPlaceTurn(double turnDeg) {

        super(
            new PIDController(TURN_kP, TURN_kI, TURN_kD),
            NAR_Drivetrain.getInstance()::getHeading,
            0, // setpoint initialized in initalize
            output -> NAR_Drivetrain.getInstance().tankDrive(output + Math.copySign(TURN_kF, output), -output - Math.copySign(TURN_kF, output)),
            NAR_Drivetrain.getInstance()
        );

        this.drivetrain = NAR_Drivetrain.getInstance();
        this.turnDeg = turnDeg;

        addRequirements(drivetrain);
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(TURN_TOLERANCE);
    }

    @Override
    public void initialize() {
        super.initialize();

        // Hack to make sure the robot turns turnDeg degrees from current heading and not turnDeg degrees from 0
        double setpoint = MathUtil.inputModulus(drivetrain.getHeading() + turnDeg, -180, 180);
        m_setpoint = () ->  setpoint;
    }

    public boolean isFinished() {
        return getController().atSetpoint();
    }

}
