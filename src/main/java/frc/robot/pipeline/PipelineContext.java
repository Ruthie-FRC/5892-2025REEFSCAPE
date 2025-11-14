package frc.robot.pipeline;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * PipelineContext holds all the data needed for driving filters.
 * Filters will read from this context and may update output commands.
 *
 * This is the superset of everything a filter might need:
 * - Joystick inputs (x, y, rotation)
 * - Robot state (pose, heading)
 * - Computed vectors (heading, velocity)
 * - Fencing info (distance to physical walls / allowed areas)
 * - Filter outputs
 *
 * This allows filters to remain decoupled and pluggable.
 */
public class PipelineContext {

    // -----------------------
    // Joystick / driver input
    // -----------------------
    
    /** X-axis of the joystick [-1, 1], positive = forward. */
    public double joystickX = 0;

    /** Y-axis of the joystick [-1, 1], positive = right. */
    public double joystickY = 0;

    /** Rotational input from joystick [-1, 1], positive = clockwise. */
    public double joystickRot = 0;

    // -----------------------
    // Field-relative robot state
    // -----------------------
    
    /** Robot position on field (meters). */
    public Translation2d robotPosition = new Translation2d(0, 0);

    /** Robot heading on the field. */
    public Rotation2d robotHeading = new Rotation2d();

    /** Current robot chassis speeds (m/s for x/y, rad/s for rotation). */
    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    // -----------------------
    // Computed vectors
    // -----------------------
    
    /** Desired movement vector in field coordinates, pre-fence adjustments. */
    public Translation2d desiredVector = new Translation2d(0, 0);

    /** Magnitude of the desired movement vector (m/s). */
    public double desiredSpeed = 0;

    /** Angle of desired movement vector relative to field forward (radians). */
    public double desiredAngle = 0;

    // -----------------------
    // Fencing / wall info
    // -----------------------
    
    /** Distance to nearest wall/fence in meters. */
    public double distanceToFence = Double.MAX_VALUE;

    /** Angle to the nearest fence relative to robot forward (radians). */
    public double angleToFence = 0;

    /** Maximum allowed speed towards the fence (m/s) */
    public double maxSpeedTowardsFence = Double.MAX_VALUE;

    /** Whether the robot is currently inside a restricted zone. */
    public boolean insideFence = false;

    // -----------------------
    // Filter output (modifiable)
    // -----------------------
    
    /** Final X speed after all filters applied (m/s) */
    public double outputX = 0;

    /** Final Y speed after all filters applied (m/s) */
    public double outputY = 0;

    /** Final rotational speed after all filters applied (rad/s) */
    public double outputRot = 0;

    // -----------------------
    // Configuration / tuning
    // -----------------------
    
    /** TODO: configurable start distance for fence gradient (meters). */
    public double fenceGradientStart = 1.5; // 1.5 meters from wall by default

    /** TODO: max allowed speed near fence (towards wall) in m/s. */
    public double fenceMaxSpeed = 1.0;

    /** TODO: whether to use strict boundary or gradient mode */
    public boolean useGradient = true;

    /** TODO: rate limiter values (max acceleration, m/s^2) */
    public double maxAccel = 3.0;

    /** TODO: rate limiter values (max rotational acceleration, rad/s^2) */
    public double maxRotAccel = Math.toRadians(180);

    /**
     * Resets the output fields to match the current desired vector.
     * This is useful at the start of pipeline processing.
     */
    public void resetOutputs() {
        outputX = desiredVector.getX();
        outputY = desiredVector.getY();
        outputRot = joystickRot;
    }
}
