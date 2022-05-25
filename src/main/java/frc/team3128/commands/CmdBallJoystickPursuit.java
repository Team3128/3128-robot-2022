package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.DriveConstants;
import static frc.team3128.Constants.VisionConstants.*;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdBallJoystickPursuit extends CommandBase {

    private enum BallPursuitState {
        SEARCHING, FEEDBACK, BLIND;
    }

    private NAR_Drivetrain drive;
    private LimelightSubsystem limelights;
    private Limelight ballLimelight;

    private double previousVerticalAngle;
    private double approxDistance;

    private double currentError, previousError;
    private double currentTime, previousTime;

    private DoubleSupplier xSupplier, ySupplier, throttleSupplier;

    private int targetCount, plateauCount;

    private BallPursuitState aimState = BallPursuitState.SEARCHING;

    
    public CmdBallJoystickPursuit(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier throttleSupplier) {
        
        drive = NAR_Drivetrain.getInstance();
        limelights = LimelightSubsystem.getInstance();
        ballLimelight = limelights.getBallLimelight();
        
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.throttleSupplier = throttleSupplier;

        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        switch (aimState) {
            case SEARCHING:
                if (ballLimelight.hasValidTarget()) {
                    targetCount++;
                } else {
                    targetCount = 0;
                    Log.info("CmdBallJoystickPursuit", "No targets ... switching to BLIND...");
                    aimState = BallPursuitState.BLIND;
                }

                if (targetCount > BALL_THRESHOLD) {
                    Log.info("CmdBallJoystickPursuit", "Target found, switching to FEEDBACK.");
                    
                    double currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    previousTime = RobotController.getFPGATime() / 1e6; 
                    previousError = GOAL_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    aimState = BallPursuitState.FEEDBACK;
                }

                break;
            
            case FEEDBACK:
                if (!ballLimelight.hasValidTarget()) {
                    Log.info("CmdBallJoystickPursuit", "No valid target anymore.");
                    aimState = BallPursuitState.SEARCHING;
                } 
                else {
                    double currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    currentTime = RobotController.getFPGATime() / 1e6; 
                    currentError = currentHorizontalOffset;

                    // PID feedback loop for left+right powers based on horizontal offset errors
                    double turnPower = 0;
                    
                    turnPower += BALL_VISION_kP * currentError;
                    turnPower += BALL_VISION_kD * (currentError - previousError) / (currentTime - previousTime);

                    turnPower = MathUtil.clamp(turnPower, -1, 1);

                    double xPower = MathUtil.clamp(
                        BALL_AUTO_PURSUIT_kF + 
                        xSupplier.getAsDouble() * throttleSupplier.getAsDouble()
                        , -1, 1);
                    
                    // calculations to decelerate as the robot nears the target

                    // approxDistance = ballLimelight.calculateDistToGroundTarget(BALL_TARGET_HEIGHT / 2);
                    // multiplier = 1.0 - Math.min(Math.max((BALL_DECELERATE_START_DISTANCE - approxDistance)
                    //         / (BALL_DECELERATE_START_DISTANCE - 
                    //             BALL_DECELERATE_END_DISTANCE), 0.0), 1.0);

                    double multiplier = POWER_MULTIPLIER * 1.0;

                    drive.arcadeDrive(xPower * multiplier, turnPower * POWER_MULTIPLIER);

                    previousTime = currentTime;
                    previousError = currentError;
                }
                break;
            
            case BLIND:
                double throttle = throttleSupplier.getAsDouble();
                double x = xSupplier.getAsDouble();
                double y = DriveConstants.ARCADE_DRIVE_TURN_MULT * ySupplier.getAsDouble();

                drive.arcadeDrive(x * throttle, y * throttle);

                if (ballLimelight.hasValidTarget()) {
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