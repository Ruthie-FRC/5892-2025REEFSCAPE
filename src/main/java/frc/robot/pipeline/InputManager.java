package frc.robot.pipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * InputManager
 *
 * Builds and executes the robot's input-processing pipeline.
 *
 * Responsibilities:
 *   - Holds an ordered list of Filter instances
 *   - Creates a fresh PipelineContext each frame from raw inputs
 *   - Applies only the filters allowed by FilterToggles
 *   - Returns the fully-processed context to the drivetrain or caller
 *
 * Behavior rules:
 *   - Filters run in the exact order they are added
 *   - Toggles override filter activity (disableAll/enableAll)
 *   - InputManager does NOT store raw joystick objects
 *   - InputManager does NOT create filters on its own
 *
 * TODO: Support driver profile switching
 * TODO: Support reorderable filter chains at runtime
 * TODO: Add telemetry hooks for debugging and tuning
 */
public class InputManager {

    /** Ordered pipeline filters */
    private final List<Filter> filters = new ArrayList<>();

    /** Toggle registry controlling which filters are active */
    private final FilterToggles toggles;

    /** Last processed context */
    private PipelineContext lastContext;

    /**
     * Constructor.
     *
     * @param toggles Shared toggle object used to enable/disable filters globally.
     */
    public InputManager(FilterToggles toggles) {
        this.toggles = toggles;

        // Initialize last context with zero-motion defaults.
        lastContext = new PipelineContext();
    }

    /**
     * Add a filter to the pipeline.
     *
     * Filters run in the exact order added.
     *
     * @param filter Implementation of Filter interface.
     */
    public void addFilter(Filter filter) {
        filters.add(filter);
        filter.initialize();
    }

    /**
     * Removes a filter instance from the pipeline.
     *
     * @param filter Filter to remove
     */
    public void removeFilter(Filter filter) {
        filter.cleanup();
        filters.remove(filter);
    }

    /**
     * Process joystick input through the full filter pipeline.
     *
     * @param joystickX Lateral joystick input (meters/sec, +left)
     * @param joystickY Forward joystick input (meters/sec, +forward)
     * @param rotation  Rotation rate (rad/s, +CCW)
     * @param heading   Robot heading (radians from field forward)
     * @return Completed PipelineContext
     */
    public PipelineContext process(double joystickX, double joystickY, double rotation, double heading) {

        // Build fresh raw context
        PipelineContext context = new PipelineContext(
            joystickX,
            joystickY,
            rotation,
            heading
        );

        // Apply filters in sequence
        for (Filter filter : filters) {
            if (isFilterEnabled(filter)) {
                context = filter.apply(context);
            }
        }

        lastContext = context;
        return context;
    }

    /**
     * Check whether a given filter should run based on toggles.
     * Uses the filter's name to choose the proper toggle boolean.
     *
     * @param filter Filter instance
     * @return true if active, false if skipped
     */
    private boolean isFilterEnabled(Filter filter) {
        String name = filter.getName();

        if (toggles.disableAll) return false;
        if (toggles.enableAll) return true;

        switch (name) {

            case "GeofenceFilter":
                return toggles.geofence;

            case "RateLimiterFilter":
                return toggles.rateLimiter;

            case "SpeedLimiterFilter":
                return toggles.speedLimiter;

            case "AntiTipFilter":
                return toggles.antiTip;

            case "InputSmoothingFilter":
                return toggles.inputSmoothing;

            default:
                // TODO: Handle newly-added filters gracefully
                return false;
        }
    }

    /**
     * Returns the last fully processed context.
     */
    public PipelineContext getLastContext() {
        return lastContext;
    }
}
