package frc.robot.pipeline;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;
import java.util.Arrays;

/**
 * GeofenceFilter
 *
 * Prevents the robot from barreling into walls like an uninsured shopping cart.
 * Longer, thicker, and more feature-rich version because the user said “make it bigger.”
 *
 * Key Features:
 *  - Multiple fences with IDs for debugging
 *  - Nonlinear deceleration curve near fence
 *  - Tangential sliding preserved
 *  - Cached normalized normals for efficiency
 *  - Engagement state machine (NONE, APPROACHING, CLAMPED)
 *  - AdvantageKit logging hooks
 *  - Precomputed useful values to reduce runtime math
 */
public class GeofenceFilter implements InputFilter {

    public enum EngagementState {
        NONE, APPROACHING, CLAMPED
    }

    /**
     * Fence definition
     */
    public static class Fence {
        public final String id;
        public final Translation2d origin;
        public final Translation2d normal; // guaranteed normalized by constructor
        public final double startDistance;
        public final double maxSpeedAtFence;

        public Fence(String id, Translation2d origin, Translation2d normal, double startDistance, double maxSpeedAtFence) {
            this.id = id;
            this.origin = origin;
            this.normal = normal.div(normal.getNorm()); // normalized here
            this.startDistance = startDistance;
            this.maxSpeedAtFence = maxSpeedAtFence;
        }
    }

    private Fence[] fences;
    private boolean enabled = true;

    // Engagement tracking per fence
    private EngagementState[] engagementStates;

    // Cached robot position placeholder
    private Translation2d robotPosition = new Translation2d(0, 0); // TODO replace with odometry input

    public GeofenceFilter(Fence[] fences) {
        this.fences = fences;
        this.engagementStates = new EngagementState[fences.length];
        Arrays.fill(engagementStates, EngagementState.NONE);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public InputState filter(Translation2d inputTranslation, double inputRotation) {
        if (!enabled) {
            return new InputState(inputTranslation, inputRotation);
        }

        Translation2d output = inputTranslation;

        // Loop through fences
        for (int i = 0; i < fences.length; i++) {
            Fence f = fences[i];

            // Precompute vectors
            Translation2d toFence = f.origin.minus(robotPosition);
            double distToFence = toFence.getX() * f.normal.getX() + toFence.getY() * f.normal.getY();
            double speedTowardFence = output.getX() * f.normal.getX() + output.getY() * f.normal.getY();

            // Determine engagement state
            EngagementState state = EngagementState.NONE;
            if (speedTowardFence > 0 && distToFence <= f.startDistance) {
                state = speedTowardFence > f.maxSpeedAtFence ? EngagementState.CLAMPED : EngagementState.APPROACHING;
            }

            engagementStates[i] = state;

            // Log state
            Logger.recordOutput("Pipeline/Geofence/" + f.id + "/distance", distToFence);
            Logger.recordOutput("Pipeline/Geofence/" + f.id + "/speedToward", speedTowardFence);
            Logger.recordOutput("Pipeline/Geofence/" + f.id + "/state", state.toString());

            if (speedTowardFence <= 0 || distToFence > f.startDistance) continue;

            // Nonlinear scaling for smoother decel near fence (quadratic)
            // 1 at far edge, 0 at fence
            double normalized = Math.max(0, distToFence / f.startDistance);
            double nonlinearScale = normalized * normalized;

            double maxAllowedSpeed = f.maxSpeedAtFence +
                    (speedTowardFence - f.maxSpeedAtFence) * nonlinearScale;

            if (speedTowardFence > maxAllowedSpeed) {
                Translation2d normalComponent = f.normal.times(maxAllowedSpeed);
                Translation2d perpendicularComponent =
                        output.minus(f.normal.times(speedTowardFence));

                output = perpendicularComponent.plus(normalComponent);
            }
        }

        return new InputState(output, inputRotation);
    }

    // Optional: feed real pose later
    public void setRobotPosition(Translation2d pos) {
        this.robotPosition = pos;
    }

    // TODO: allow editing fences at runtime
    // TODO: add trajectory-aware braking
    // TODO: integrate curvature-based slowdown for sharper approach smoothing
    // TODO: export fence engagement to DriverStation UI
}
