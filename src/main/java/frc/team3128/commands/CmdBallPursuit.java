package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.VisionConstants.*;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdBallPursuit extends CommandBase {

    private enum BallPursuitState {
        SEARCHING, FEEDBACK, BLIND;
    }

    private NAR_Drivetrain drive;
    private LimelightSubsystem limelights;

    private double previousVerticalAngle;

    private double currentError, previousError;
    private double currentTime, previousTime;
    private double currentHorizontalOffset;
    private double multiplier, turnPower;
    private double approxDistance;

    int targetCount, plateauCount;

    private BallPursuitState aimState = BallPursuitState.SEARCHING;

    
    public CmdBallPursuit() {
        drive = NAR_Drivetrain.getInstance();
        limelights = LimelightSubsystem.getInstance();

        addRequirements(drive);
    }

    @Override
    public void execute() {
        Log.info("State", aimState.toString());
        switch (aimState) {
            case SEARCHING:
                if (limelights.getBallHasValidTarget()) {
                    targetCount++;
                } else {
                    targetCount = 0;
                    Log.info("CmdBallJoystickPursuit", "No targets ... switching to BLIND...");
                    aimState = BallPursuitState.BLIND;
                }

                if (targetCount > BALL_THRESHOLD) {
                    Log.info("CmdBallJoystickPursuit", "Target found, switching to FEEDBACK.");
                    
                    currentHorizontalOffset = limelights.getBallTX();

                    previousTime = RobotController.getFPGATime() / 1e6; 
                    previousError = GOAL_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    aimState = BallPursuitState.FEEDBACK;
                }

                break;
            
            case FEEDBACK:
                if (!limelights.getBallHasValidTarget()) {
                    Log.info("CmdBallPursuit", "No valid target anymore.");
                    aimState = BallPursuitState.SEARCHING;
                } else {
                    currentHorizontalOffset = limelights.getBallTX();
                    
                    currentTime = RobotController.getFPGATime() / 1e6; 
                    currentError = currentHorizontalOffset;

                    // PID feedback loop for left+right powers based on horizontal offset errors
                    turnPower = 0;
                    
                    turnPower += BALL_VISION_kP * currentError;
                    turnPower += BALL_VISION_kD * (currentError - previousError) / (currentTime - previousTime);

                    turnPower = MathUtil.clamp(turnPower, -1, 1);
                    
                    // calculations to decelerate as the robot nears the target
                    approxDistance = limelights.calculateBallDistance();

                    multiplier = 1.0 - Math.min(Math.max((BALL_DECELERATE_START_DISTANCE - approxDistance)
                            / (BALL_DECELERATE_START_DISTANCE - 
                                BALL_DECELERATE_END_DISTANCE), 0.0), 1.0);                    

                    drive.arcadeDrive(BALL_VISION_kF * multiplier * POWER_MULTIPLIER, turnPower * POWER_MULTIPLIER);
                    previousTime = currentTime;
                    previousError = currentError;
                }
                break;
            
            case BLIND:

                drive.tankDrive(0.35, 0.35); // drive straight 

                if (limelights.getBallHasValidTarget()) {
                    Log.info("CmdBallPursuit", "Target found - Switching to SEARCHING");
                    aimState = BallPursuitState.SEARCHING;
                }
                    
                break;
        }
    }

    @Override
    public boolean isFinished() {
        /*
        When the robot is moving very slowly + blind the robot has probably just intook (?)
        since it decelerated and there is no more target the limelight sees.
        */
        if (aimState == BallPursuitState.BLIND) {
            double leftVel = Math.abs(drive.getLeftEncoderSpeed());
            double rightVel = Math.abs(drive.getRightEncoderSpeed());

            if (leftVel < BALL_VEL_THRESHOLD && rightVel < BALL_VEL_THRESHOLD) {
                plateauCount += 1;
            } else {
                plateauCount = 0;
            }

            if (plateauCount >= BALL_VEL_PLATEAU_THRESHOLD) {
                return false;
            }
        }

        // if in feedback and only 1 ball length away, stop command & give control back to teleop (but not if ball is bouncing and messing up the math)
        if (aimState == BallPursuitState.FEEDBACK) {
            approxDistance = limelights.calculateBallDistance();
            if (BALL_DECELERATE_END_DISTANCE > approxDistance && previousVerticalAngle < 20*Math.PI/180) {
                Log.info("CmdBallPursuit", "decelerated! ending command now");
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();

        Log.info("CmdBallPursuit", "Command Ended.");
    }

}