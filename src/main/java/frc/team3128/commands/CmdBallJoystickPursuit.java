package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.DriveConstants.*;
import static frc.team3128.Constants.VisionConstants.*;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdBallJoystickPursuit extends CommandBase {

    private enum BallPursuitState {
        SEARCHING, FEEDBACK, BLIND;
    }

    private NAR_Drivetrain drive;
    private LimelightSubsystem limelights;

    private double approxDistance;

    private double currentError, previousError;
    private double currentTime, previousTime;
    private double currentHorizontalOffset;

    private DoubleSupplier xSupplier, ySupplier, throttleSupplier;

    private int targetCount;

    private BallPursuitState aimState = BallPursuitState.SEARCHING;

    private SlewRateLimiter filter = new SlewRateLimiter(ARCADE_DRIVE_RATE_LIMIT);

    /**
     * Aligns robot to ball and pursuits autonomously and with joystick input
     * 
     * Functionally same as CmdArcadeDrive if no target spotted
     * @Requirements Drivetrain
     */
    public CmdBallJoystickPursuit(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier throttleSupplier) {
        
        drive = NAR_Drivetrain.getInstance();
        limelights = LimelightSubsystem.getInstance();
        
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.throttleSupplier = throttleSupplier;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        switch (aimState) {
            case SEARCHING:
                // if no target, switch back to BLIND
                if (limelights.getBallHasValidTarget()) {
                    targetCount++;
                } else {
                    targetCount = 0;
                    Log.info("CmdBallJoystickPursuit", "No targets ... switching to BLIND...");
                    aimState = BallPursuitState.BLIND;
                }

                // if targets found for enough iterations, switch to FEEDBACK
                if (targetCount > BALL_THRESHOLD) {
                    Log.info("CmdBallJoystickPursuit", "Target found, switching to FEEDBACK.");
                    
                    currentHorizontalOffset = limelights.getBallTX();

                    previousTime = RobotController.getFPGATime() / 1e6; 
                    previousError = GOAL_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    aimState = BallPursuitState.FEEDBACK;
                }

                break;
            
            case FEEDBACK:
                // if no target, switch to SEARCHING
                if (!limelights.getBallHasValidTarget()) {
                    Log.info("CmdBallJoystickPursuit", "No valid target anymore.");
                    aimState = BallPursuitState.SEARCHING;
                } 
                else {
                    // PID feedback loop for turning power based on horizontal tx error
                    currentHorizontalOffset = limelights.getBallTX();

                    currentTime = RobotController.getFPGATime() / 1e6; 
                    currentError = currentHorizontalOffset;

                    double turnPower = 0;
                    
                    turnPower += BALL_VISION_kP * currentError;
                    turnPower += BALL_VISION_kD * (currentError - previousError) / (currentTime - previousTime);

                    turnPower = MathUtil.clamp(turnPower, -1, 1);

                    // calculate x (forward/backward) power based on joystick input + constant FF
                    double xPower = MathUtil.clamp(
                        BALL_AUTO_PURSUIT_kF + 
                        xSupplier.getAsDouble() * throttleSupplier.getAsDouble()
                        , -1, 1);
                    
                    // calculations to decelerate as the robot nears the target

                    // approxDistance = limelights.calculateBallDistance();
                    // multiplier = 1.0 - Math.min(Math.max((BALL_DECELERATE_START_DISTANCE - approxDistance)
                    //         / (BALL_DECELERATE_START_DISTANCE - 
                    //             BALL_DECELERATE_END_DISTANCE), 0.0), 1.0);

                    double multiplier = POWER_MULTIPLIER * 1.0;

                    drive.arcadeDrive(filter.calculate(xPower * multiplier), turnPower * POWER_MULTIPLIER);

                    previousTime = currentTime;
                    previousError = currentError;
                }
                break;
            
            case BLIND:
                // if no target, let driver have regular drive control of robot
                double throttle = throttleSupplier.getAsDouble();
                double x = xSupplier.getAsDouble();
                double y = ARCADE_DRIVE_TURN_MULT * ySupplier.getAsDouble();

                drive.arcadeDrive(filter.calculate(x * throttle), y);

                if (limelights.getBallHasValidTarget()) {
                    Log.info("CmdBallJoystickPursuit", "Target found - Switching to SEARCHING");
                    aimState = BallPursuitState.SEARCHING;
                }
                    
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();

        Log.info("CmdBallJoystickPursuit", "Command Ended.");
    }

}