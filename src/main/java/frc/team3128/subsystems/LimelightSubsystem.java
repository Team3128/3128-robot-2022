package frc.team3128.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.limelight.LEDMode;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;

import static frc.team3128.Constants.VisionConstants.*;

/**
 * Class for the Limelight Subsystem 
 */

public class LimelightSubsystem extends SubsystemBase{

    private static LimelightSubsystem instance;
    private Limelight m_shooterLimelight;
    private Limelight m_ballLimelight;

    static {
        instance = new LimelightSubsystem();
    }

    private LimelightSubsystem() {
        m_shooterLimelight = new Limelight("limelight-cog", TOP_CAMERA_ANGLE, TOP_CAMERA_HEIGHT, TOP_FRONT_DIST); 
        m_ballLimelight = new Limelight("limelight-sog", BALL_LL_ANGLE, BALL_LL_HEIGHT, BALL_LL_FRONT_DIST);
    }

    public static LimelightSubsystem getInstance() {
        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("range", calculateShooterDistance());
        SmartDashboard.putNumber("ty", getShooterTY());
        SmartDashboard.putNumber("tx", getShooterTX());
        SmartDashboard.putBoolean("hasValidTarget", getShooterHasValidTarget());
    }

    /**
     * Wrapper function to uniformly calculate distance to a elevated target using a limelight
     * For use: extra calculations outside of the base one should happen
     * in this function (eg: interpolation to get accurate distance)
     */
    public double calculateShooterDistance() {
        // we need to redo interpolation
        return m_shooterLimelight.calculateDistToTopTarget(TARGET_HEIGHT) + 10;
    }

    /**
     * Wrapper function to uniformly calculate distance to a ground target using a limelight
     */
    public double calculateBallDistance() {
        return m_ballLimelight.calculateDistToGroundTarget(BALL_TARGET_HEIGHT / 2);
    }

    /**
     * Wrapper function to get shooter horizontal offset (tx) to target
     */
    public double getShooterTX() {
        return m_shooterLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET);
    }

    /**
     * Wrapper function to get shooter vertical offset (ty) to target
     */
    public double getShooterTY() {
        return m_shooterLimelight.getValue(LimelightKey.VERTICAL_OFFSET);
    }

    /**
     * Wrapper function to get if the shooter limelight has a valid target
     */
    public boolean getShooterHasValidTarget() {
        return m_shooterLimelight.hasValidTarget();
    }

    /**
     * Wrapper function to get ball horizontal offset (tx) to target
     */
    public double getBallTX() {
        return m_ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET);
    }

    /**
     * Wrapper function to get ball vertical offset (ty) to target
     */
    public double getBallTY() {
        return m_ballLimelight.getValue(LimelightKey.VERTICAL_OFFSET);
    }

    /**
     * Wrapper function to get if the ball limelight has a valid target
     */
    public boolean getBallHasValidTarget() {
        return m_ballLimelight.hasValidTarget();
    }
    
    /**
     * Returns top-facing limelight object
     * @return shooterLimelight object
     */
    public Limelight getShooterLimelight() {
        return m_shooterLimelight;
    }

    /**
     * Returns bottom-facing limelight object
     * @return ballLimelight object
     */
    public Limelight getBallLimelight() {
        return m_ballLimelight;
    }
    
    public void turnShooterLEDOff() {
        m_shooterLimelight.setLEDMode(LEDMode.OFF);
    }

    public void turnShooterLEDOn() {
        m_shooterLimelight.setLEDMode(LEDMode.ON);
    }

}
