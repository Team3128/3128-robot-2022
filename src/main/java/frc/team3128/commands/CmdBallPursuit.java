package frc.team3128.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants;
import frc.team3128.common.hardware.limelight.LEDMode;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdBallPursuit extends CommandBase {
    private final NAR_Drivetrain m_drivetrain;

    private final Limelight ballLimelight;

    private double multiplier;

    private double currentHorizontalOffset;
    private double previousVerticalAngle;
    private double approxDistance;

    private double currentError, previousError;
    private double currentTime, previousTime;

    private double feedbackPower;
    private double leftVel, rightVel;

    int targetCount, plateauCount;

    private enum BallPursuitState {
        SEARCHING, FEEDBACK, BLIND;
    }

    private BallPursuitState aimState = BallPursuitState.SEARCHING;
    
    public CmdBallPursuit(NAR_Drivetrain drive, Limelight ballLimelight) {
        this.ballLimelight = ballLimelight;
        m_drivetrain = drive;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        ballLimelight.setLEDMode(LEDMode.ON);
    }

    @Override
    public void execute() {
        Log.info("State", aimState.toString());
        switch (aimState) {
            case SEARCHING:
                if (ballLimelight.hasValidTarget()) {
                    targetCount ++;
                } else {
                    targetCount = 0;
                    Log.info("CmdBallPursuit", "No targets ... switching to BLIND...");
                    aimState = BallPursuitState.BLIND;
                }

                if (targetCount > Constants.VisionContants.BALL_THRESHOLD) {
                    Log.info("CmdBallPursuit", "Target found.");
                    Log.info("CmdBallPursuit", "Switching to FEEDBACK...");
                    
                    currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    previousTime = RobotController.getFPGATime() / 1e6; 
                    previousError = Constants.VisionContants.GOAL_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    aimState = BallPursuitState.FEEDBACK;
                }

                break;
            
            case FEEDBACK:
                if (!ballLimelight.hasValidTarget()) {
                    Log.info("CmdBallPursuit", "No valid target anymore.");
                    aimState = BallPursuitState.SEARCHING;
                } else {
                    currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    currentTime = RobotController.getFPGATime() / 1e6; 
                    currentError = Constants.VisionContants.GOAL_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    // PID feedback loop for left+right powers based on horizontal offset errors
                    feedbackPower = 0;
                    
                    feedbackPower += Constants.VisionContants.BALL_VISION_kP * currentError;
                    feedbackPower += Constants.VisionContants.BALL_VISION_kD * (currentError - previousError) / (currentTime - previousTime);
                    
                    feedbackPower = Math.min(Math.max(feedbackPower, -1), 1);
                    
                    // calculations to decelerate as the robot nears the target
                    previousVerticalAngle = ballLimelight.getValue(LimelightKey.VERTICAL_OFFSET, 2) * Math.PI / 180;
                    approxDistance = ballLimelight.calculateDistToGroundTarget(previousVerticalAngle, Constants.VisionContants.BALL_TARGET_HEIGHT / 2);

                    multiplier = 1.0 - Math.min(Math.max((Constants.VisionContants.BALL_DECELERATE_START_DISTANCE - approxDistance)
                            / (Constants.VisionContants.BALL_DECELERATE_START_DISTANCE - 
                                Constants.VisionContants.BALL_DECELERATE_END_DISTANCE), 0.0), 1.0);

                    m_drivetrain.arcadeDrive(0.7*Constants.VisionContants.BALL_VISION_kF*multiplier, 0.7*feedbackPower);
                    previousTime = currentTime;
                    previousError = currentError;
                }
                break;
            
            case BLIND:

                m_drivetrain.tankDrive(0.35, -0.35);

                if (ballLimelight.hasValidTarget()) {
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
            leftVel = Math.abs(m_drivetrain.getLeftEncoderSpeed());
            rightVel = Math.abs(m_drivetrain.getRightEncoderSpeed());

            if (leftVel < Constants.VisionContants.BALL_VEL_THRESHOLD && rightVel < Constants.VisionContants.BALL_VEL_THRESHOLD) {
                plateauCount += 1;
            } else {
                plateauCount = 0;
            }

            if (plateauCount >= Constants.VisionContants.BALL_VEL_PLATEAU_THRESHOLD) {
                return false;
            }
        }

        // if in feedback and only 1 ball length away, stop command & give control back to teleop (but not if ball is bouncing and messing up the math)
        if (aimState == BallPursuitState.FEEDBACK) {
            previousVerticalAngle = ballLimelight.getValue(LimelightKey.VERTICAL_OFFSET, 2) * Math.PI / 180;
            approxDistance = ballLimelight.calculateDistToGroundTarget(previousVerticalAngle, Constants.VisionContants.BALL_TARGET_HEIGHT / 2);
            if (Constants.VisionContants.BALL_DECELERATE_END_DISTANCE > approxDistance && previousVerticalAngle < 20*Math.PI/180) {
                Log.info("CmdBallPursuit", "decelerated! ending command now");
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
        ballLimelight.setLEDMode(LEDMode.OFF);

        Log.info("CmdBallPursuit", "Command Ended.");
    }

}