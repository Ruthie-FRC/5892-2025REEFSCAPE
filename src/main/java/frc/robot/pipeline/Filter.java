package frc.robot.pipeline;

/**
 * Base interface for all filters in the robot input pipeline.
 * 
 * Each filter takes in a {@link PipelineContext} object (which contains joystick inputs,
 * velocity commands, robot heading, and other sensor data) and modifies it according to
 * the filter's logic. 
 * 
 * This allows filters to be chained together in a pluggable, modular pipeline.
 */
public interface Filter {

    /**
     * Process the inputs in the provided {@link PipelineContext}.
     * 
     * @param context The pipeline context containing all current robot inputs and state.
     * @return The modified context after applying this filter.
     */
    PipelineContext apply(PipelineContext context);

    /**
     * Optional method for initialization or resetting state.
     * Called when the robot code first starts or when the driver profile changes.
     */
    default void initialize() {
        // Default: no initialization needed
    }

    /**
     * Optional method for cleanup if the filter allocates resources.
     */
    default void cleanup() {
        // Default: nothing to clean
    }

    /**
     * Each filter should provide a descriptive name.
     * Useful for logging or debugging.
     * 
     * @return Name of the filter.
     */
    default String getName() {
        return this.getClass().getSimpleName();
    }
}
