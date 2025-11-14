package frc.robot.pipeline;

/**
 * FilterToggles
 *
 * Centralized enable/disable registry for all pipeline filters.
 * InputManager will read these booleans and decide which filters
 * to construct and run during the pipeline loop.
 *
 * This class contains:
 *   - Individual toggles for each filter
 *   - A master "enable all" toggle
 *   - A master "disable all" toggle
 *   - Utility methods for resetting or overriding all toggles at once
 *
 * IMPORTANT:
 *   This class DOES NOT directly call any filters.
 *   It simply exposes boolean flags. InputManager decides how and when
 *   to assemble filter chains based on this info.
 */
public final class FilterToggles {

    /** Enables ALL filters at once. Overrides individual toggles. */
    public boolean enableAll = false;

    /** Disables ALL filters at once. Overrides individual toggles. */
    public boolean disableAll = false;

    // -----------------------------
    // INDIVIDUAL FILTER TOGGLES
    // -----------------------------

    /** Enable or disable the geofence speed limiter. */
    public boolean geofence = false;

    /** Enable or disable rate-limiting (acceleration limiter). */
    public boolean rateLimiter = false;

    /** Enable or disable speed limiting based on max driver preference. */
    public boolean speedLimiter = false;

    /** Enable or disable IMU-based anti-tip filter. */
    public boolean antiTip = false;

    /** Enable or disable joystick deadbands / noise cleaning filter. */
    public boolean inputSmoothing = false;

    // TODO: Add toggle for driver-specific profile selection
    // TODO: Add toggle for dynamic filter loading via AdvantageScope or NetworkTables
    // TODO: Add toggle for "sandbox testing mode" for rapid filter debugging

    // -----------------------------
    // CONVENIENCE METHODS
    // -----------------------------

    /**
     * Disable every filter, including master flags.
     * Useful when testing raw joystick behavior.
     */
    public void disableEverything() {
        enableAll = false;
        disableAll = true;

        geofence = false;
        rateLimiter = false;
        speedLimiter = false;
        antiTip = false;
        inputSmoothing = false;
    }

    /**
     * Enable every filter except the ones intentionally dangerous
     * or marked as experimental. This obeys the per-filter booleans
     * unless enableAll is set.
     *
     * WARNING:
     * If enableAll is true, InputManager will force all filters on
     * regardless of the status of individual flags.
     */
    public void enableEverything() {
        enableAll = true;
        disableAll = false;

        geofence = true;
        rateLimiter = true;
        speedLimiter = true;
        antiTip = true;
        inputSmoothing = true;
    }

    /**
     * Reset all toggles to the default "all off" safe state.
     */
    public void reset() {
        enableAll = false;
        disableAll = false;

        geofence = false;
        rateLimiter = false;
        speedLimiter = false;
        antiTip = false;
        inputSmoothing = false;
    }

    /**
     * Returns true if a filter with a given boolean
     * should be active, after considering master flags.
     *
     * InputManager should call this when building the pipeline.
     *
     * @param individualToggle the filter’s individual boolean flag
     * @return true if filter should run, false otherwise
     */
    public boolean isActive(boolean individualToggle) {
        if (disableAll) return false;
        if (enableAll) return true;
        return individualToggle;
    }
}
