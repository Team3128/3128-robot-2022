package frc.team3128.subsystems;

public class Constants {
    public static class SidekickConstants {
        public static final float SIDEKICK_PID_kP = 0;
        public static final float SIDEKICK_PID_kI = 0;
        public static final float SIDEKICK_PID_kD = 0;
        public static final int SIDEKICK_RPM_THRESHOLD_PERCENT = 0;
        public static final int SIDEKICK_ID = 0;
        public static final int CAN_TIMEOUT = 0;
        public static final double SIDEKICK_kS = 0;
        public static final double SIDEKICK_kV = 0;
    }

    public static class ConversionConstants {
        public static final double SIDEKICK_ENCODER_TO_RPM = 10*60/MechanismConstants.SIDEKICK_UNITS_PER_ROTATION;
    }

    public static class MechanismConstants {
        public static final double SIDEKICK_UNITS_PER_ROTATION = 4096;
    }
}
