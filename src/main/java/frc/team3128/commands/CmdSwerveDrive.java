package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Swerve;
import static frc.team3128.Constants.SwerveConstants.*;

public class CmdSwerveDrive extends CommandBase {
    private final Swerve swerve;

    private double rotation;
    private Translation2d translation;
    
    private final boolean fieldRelative;

    private final DoubleSupplier xAxis;
    private final DoubleSupplier yAxis;
    private final DoubleSupplier zAxis;
    private final DoubleSupplier throttle;
    
    // when you call this later use getX getY getZ getThrottle
    public CmdSwerveDrive(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier zAxis, DoubleSupplier throttle, boolean fieldRelative) {
        this.swerve = Swerve.getInstance();
        addRequirements(swerve);

        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.zAxis = zAxis;
        this.throttle = throttle;
        this.fieldRelative = fieldRelative;
    }

    @Override
    public void execute() {
        // deadbands are taken care of in NAR_Joystick
        // TODO: add in slewratelimiter here
        translation = new Translation2d(xAxis.getAsDouble(), yAxis.getAsDouble()).times(throttle.getAsDouble()).times(maxSpeed);
        rotation = zAxis.getAsDouble() * maxAngularVelocity; // * throttle.getAsDouble();
        swerve.drive(translation, rotation, fieldRelative);

    }
}
