package frc.team3128.common.hardware.motorcontroller;

/**
 * Constants for motor control / conversion. Should not be changed.
 */
public class MotorControllerConstants {

    public static final double FALCON_ENCODER_RESOLUTION = 2048; // CPR
    public static final double MAG_ENCODER_RESOLUTION = 4096; // CPR

    public static final double SPARKMAX_ENCODER_RESOLUTION = 42; // CPR
    public static final double SPARKMAX_RPM_TO_NUpS = SPARKMAX_ENCODER_RESOLUTION / 60; // rotations/min -> counts/sec

}
