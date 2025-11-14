package frc.robot.pipeline;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * SpeedLimiterFilter
 * 
 * Limits the robot's translational and rotational speed according to defined
 * maximums. Can be configured per-axis and per-driver profile.
 * 
 * This filter is intended to be the first in a pipeline of input processors.
 * It ensures that even if a joystick or AI gives over-the-limit commands,
 * the robot will not exceed the specified safe velocities.
 */
public class SpeedLimiterFilter implements InputFilter {
    
    // TODO: Make these configurable via config files or driver profiles
    private final double maxForwardSpeedMetersPerSecond;   // Max forward speed
    private final double maxStrafeSpeedMetersPerSecond;    // Max sideways speed
    private final double maxRotationRadiansPerSecond;      // Max rotational speed
    
    // TODO: Optionally allow per-driver scaling factors
    private final double driverForwardScale;
    private final double driverStrafeScale;
    private final double driverRotationScale;
    
    /**
     * Constructor with maximums and default scaling of 1.0 for all axes.
     * 
     * @param maxForwardMetersPerSecond Maximum forward velocity
     * @param maxStrafeMetersPerSecond Maximum sideways velocity
     * @param maxRotationRadiansPerSecond Maximum rotation velocity
     */
    public SpeedLimiterFilter(
        double maxForwardMetersPerSecond,
        double maxStrafeMetersPerSecond,
        double maxRotationRadiansPerSecond
    ) {
        this(maxForwardMetersPerSecond, maxStrafeMetersPerSecond, maxRotationRadiansPerSecond, 1.0, 1.0, 1.0);
    }
    
    /**
     * Full constructor allowing per-axis scaling (e.g., for different driver profiles)
     * 
     * @param maxForwardMetersPerSecond Maximum forward velocity
     * @param maxStrafeMetersPerSecond Maximum sideways velocity
     * @param maxRotationRadiansPerSecond Maximum rotation velocity
     * @param driverForwardScale Driver scaling factor for forward
     * @param driverStrafeScale Driver scaling factor for strafe
     * @param driverRotationScale Driver scaling factor for rotation
     */
    public SpeedLimiterFilter(
        double maxForwardMetersPerSecond,
        double maxStrafeMetersPerSecond,
        double maxRotationRadiansPerSecond,
        double driverForwardScale,
        double driverStrafeScale,
        double driverRotationScale
    ) {
        this.maxForwardSpeedMetersPerSecond = maxForwardMetersPerSecond;
        this.maxStrafeSpeedMetersPerSecond = maxStrafeMetersPerSecond;
        this.maxRotationRadiansPerSecond = maxRotationRadiansPerSecond;
        this.driverForwardScale = driverForwardScale;
        this.driverStrafeScale = driverStrafeScale;
        this.driverRotationScale = driverRotationScale;
    }
    
    /**
     * Filters the input vector and rotation according to the limits.
     * 
     * @param inputTranslation The input translation vector from joystick
     * @param inputRotation The input rotation value (rad/s)
     * @return A filtered InputState with values clamped to limits
     */
    @Override
    public InputState filter(Translation2d inputTranslation, double inputRotation) {
        
        // Scale input by driver profile
        double scaledX = inputTranslation.getX() * driverForwardScale;
        double scaledY = inputTranslation.getY() * driverStrafeScale;
        double scaledRot = inputRotation * driverRotationScale;
        
        // Clamp each axis separately
        double clampedX = clamp(scaledX, -maxForwardSpeedMetersPerSecond, maxForwardSpeedMetersPerSecond);
        double clampedY = clamp(scaledY, -maxStrafeSpeedMetersPerSecond, maxStrafeSpeedMetersPerSecond);
        double clampedRot = clamp(scaledRot, -maxRotationRadiansPerSecond, maxRotationRadiansPerSecond);
        
        // TODO: Log these values to telemetry for tuning
        // Example: Logger.log("SpeedLimiterFilter", "X: " + clampedX + " Y: " + clampedY + " Rot: " + clampedRot);
        
        return new InputState(new Translation2d(clampedX, clampedY), clampedRot);
    }
    
    /**
     * Helper method to clamp a value between min and max.
     * 
     * @param value The value to clamp
     * @param min Minimum allowed
     * @param max Maximum allowed
     * @return The clamped value
     */
    private double clamp(double value, double min, double max) {
        if (value > max) return max;
        if (value < min) return min;
        return value;
    }
    
    // TODO: Consider adding methods for dynamic updates to max speeds during runtime
    // TODO: Add ability to combine with other filters in a pipeline automatically
}
