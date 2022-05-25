package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.limelight.LEDMode;
import frc.team3128.common.hardware.limelight.Limelight;
import static frc.team3128.Constants.VisionConstants.*;

public class LimelightSubsystem extends SubsystemBase{

    public static LimelightSubsystem instance;
    private Limelight m_shooterLimelight;
    private Limelight m_ballLimelight;

    public LimelightSubsystem() {
        m_shooterLimelight = new Limelight("limelight-cog", TOP_CAMERA_ANGLE, TOP_CAMERA_HEIGHT, TOP_FRONT_DIST); 
        m_ballLimelight = new Limelight("limelight-sog", BALL_LL_ANGLE, BALL_LL_HEIGHT, BALL_LL_FRONT_DIST);
    }

    public static synchronized LimelightSubsystem getInstance() {
        if (instance == null) {
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    /**
     * Wrapper function to uniformly calculate distance to a target using a limelight
     * @param limelight - name of limelight, "shooter" or "ball"
     */
    public double calculateDistance(String limelight) {
        if (limelight.equalsIgnoreCase("shooter")) {
            return m_shooterLimelight.calculateDistToTopTarget(TARGET_HEIGHT);
        }
        else if (limelight.equalsIgnoreCase("ball")) {
            // target height is for center of ball
            return m_ballLimelight.calculateDistToGroundTarget(BALL_TARGET_HEIGHT / 2);
        }
        return -1;
    }

    public Limelight getShooterLimelight() {
        return m_shooterLimelight;
    }

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
