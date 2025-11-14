package frc.robot.pipeline;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Holds the current input state from the driver controls.
 * All filters will use this as the input and may modify a copy for output.
 */
public class InputState {
    /** Translation vector in meters per second. Positive X = forward, Positive Y = left. */
    public final Translation2d translation;

    /** Rotation in radians per second. Positive = CCW. */
    public final double rotation;

    /** Optional: heading of the robot relative to field (0 = forward). */
    public final double fieldHeading;

    /**
     * Constructor.
     * @param translation Initial translation vector
     * @param rotation Initial rotation rate
     * @param fieldHeading Initial robot heading
     */
    public InputState(Translation2d translation, double rotation, double fieldHeading) {
        this.translation = translation;
        this.rotation = rotation;
        this.fieldHeading = fieldHeading;
    }

    /**
     * Copy constructor.
     * Useful for filters to work on a new instance without mutating the original.
     */
    public InputState(InputState other) {
        this.translation = new Translation2d(other.translation.getX(), other.translation.getY());
        this.rotation = other.rotation;
        this.fieldHeading = other.fieldHeading;
    }

    @Override
    public String toString() {
        return String.format("InputState[translation=(%.2f, %.2f) m/s, rotation=%.2f rad/s, heading=%.2f rad]",
                translation.getX(), translation.getY(), rotation, fieldHeading);
    }

    /**
     * Convenience constructor with default fieldHeading = 0
     */
    public InputState(Translation2d translation, double rotation) {
        this(translation, rotation, 0);
    }
}
