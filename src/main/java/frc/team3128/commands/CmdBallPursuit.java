package frc.team3128.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants;
import frc.team3128.common.hardware.limelight.LEDMode;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdBallPursuit extends CommandBase {
    private final NAR_Drivetrain m_drivetrain;
    private final AHRS gyro;

    private final Limelight ballLimelight;

    private double multiplier;

    private double currentHorizontalOffset;
    private double previousVerticalAngle;
    private double approxDistance;
    private double currentBlindAngle;

    private double currentError, previousError;
    private double currentTime, previousTime;

    private double feedbackPower;
    private double leftVel, rightVel;
    private double leftPower, rightPower;

    int targetCount, plateauCount;
    
    private enum BallPursuitState {
        SEARCHING, FEEDBACK, BLIND;
    }

    private BallPursuitState aimState = BallPursuitState.SEARCHING;
    
    public CmdBallPursuit(NAR_Drivetrain drive, AHRS gyro, Limelight ballLimelight) {
        this.gyro = gyro;
        this.ballLimelight = ballLimelight;
        m_drivetrain = drive;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        ballLimelight.setLEDMode(LEDMode.OFF);
    }

    @Override
    public void execute() {
        switch (aimState) {
            case SEARCHING:
                if (ballLimelight.hasValidTarget()) {
                    targetCount ++;
                } else {
                    targetCount = 0;
                    // in an ideal world this would happen after a set time but that is for after testing
                    Log.info("CmdBallPursuit", "No targets ... switching to BLIND...");
                    aimState = BallPursuitState.BLIND;
                }

                if (targetCount > Constants.VisionContants.BALL_THRESHOLD) {
                    Log.info("CmdBallPursuit", "Target found.");
                    Log.info("CmdBallPursuit", "Switching to FEEDBACK...");

                    m_drivetrain.tankDrive(0.8*Constants.VisionContants.BALL_VISION_kF, 0.8*Constants.VisionContants.BALL_VISION_kF);
                    
                    currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    previousTime = RobotController.getFPGATime() / 1e6; // CONVERT UNITS
                    previousError = Constants.VisionContants.GOAL_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    aimState = BallPursuitState.FEEDBACK;
                }

                break;
            
            case FEEDBACK:
                if (!ballLimelight.hasValidTarget()) {
                    Log.info("CmdBallPursuit", "No valid target anymore.");
                    if ((ballLimelight.cameraAngle > 0 ? 1 : -1) * previousVerticalAngle > Constants.VisionContants.BLIND_THRESHOLD) {
                        Log.info("CmdBallPursuit", "Switching to BLIND...");

                        gyro.reset();
                        aimState = BallPursuitState.BLIND;
                    } else {
                        Log.info("CmdBallPursuit", "Returning to SEARCHING...");
                        aimState = BallPursuitState.SEARCHING;
                    }
                } else {
                    currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    currentTime = RobotController.getFPGATime() / 1e6; // CONVERT UNITS
                    currentError = Constants.VisionContants.GOAL_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    // PID feedback loop for left+right powers based on horizontal offset errors
                    feedbackPower = 0;
                    
                    feedbackPower += Constants.VisionContants.BALL_VISION_kP * currentError;
                    feedbackPower += Constants.VisionContants.BALL_VISION_kD * (currentError - previousError) / (currentTime - previousTime);
                    
                    leftPower = Math.min(Math.max(Constants.VisionContants.BALL_VISION_kF - feedbackPower, -1), 1);
                    rightPower = Math.min(Math.max(Constants.VisionContants.BALL_VISION_kF + feedbackPower, -1), 1);
                    
                    previousVerticalAngle = ballLimelight.getValue(LimelightKey.VERTICAL_OFFSET, 2);

                    // calculations to decelerate as the robot nears the target
                    // unsure if the below statement is accurate - angle might need to be *-1
                    approxDistance = ballLimelight.calculateDistToGroundTarget(Math.abs(previousVerticalAngle), Constants.VisionContants.BALL_TARGET_HEIGHT / 2);

                    multiplier = 1.0 - (1.0 - Constants.VisionContants.BALL_BLIND_kF / Constants.VisionContants.BALL_VISION_kF)
                        * Math.min(Math.max((Constants.VisionContants.BALL_DECELERATE_START_DISTANCE - approxDistance)
                            / (Constants.VisionContants.BALL_DECELERATE_START_DISTANCE - 
                                Constants.VisionContants.BALL_DECELERATE_END_DISTANCE), 0.0), 1.0);

                    m_drivetrain.tankDrive(0.7*multiplier*leftPower, 0.7*multiplier*rightPower);
                    previousTime = currentTime;
                    previousError = currentError;
                }
                break;
            
            case BLIND:
                currentBlindAngle = gyro.getAngle();
                currentTime = RobotController.getFPGATime() / 1e6; // CONVERT UNITS
                currentError = -currentBlindAngle;

                // PID feedback loop for left and right powers based on gyro angle
                // this needs some work
                feedbackPower = 0;

                feedbackPower += Constants.VisionContants.BALL_BLIND_kP * currentError;
                feedbackPower += Constants.VisionContants.BALL_BLIND_kD * (currentError - previousError) / (currentTime - previousTime);

                rightPower = Math.min(Math.max(Constants.VisionContants.BALL_BLIND_kF - feedbackPower, -1), 1);
                leftPower = Math.min(Math.max(Constants.VisionContants.BALL_BLIND_kF + feedbackPower, -1), 1);

                m_drivetrain.tankDrive(0.7*leftPower, 0.7*rightPower);

                previousTime = currentTime;
                previousError = currentError;

                // in an ideal world this would have to find more than one 
                // or maybe that doesn't matter much because it will just go back to blind
                // but maybe two? need testing of initial first
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
        When the robot is moving very slowly + blind the robot has probably just
        intook (?) and since it decelerated and there is no more target the limelight sees.
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