package frc.robot.pipeline;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * RateLimiterFilter
 * 
 * Limits the rate of change of joystick inputs (both translation and rotation).
 * Prevents jerky robot motion by ramping input over time.
 * Works on full translational vectors (X/Y) and rotation separately.
 * 
 * Can be toggled on/off and supports future driver-specific profiles.
 */
public class RateLimiterFilter implements InputFilter {

    // Maximum translational acceleration (m/s^2)
    private double maxAccel; 

    // Maximum rotational acceleration (rad/s^2)
    private double maxAngularAccel;

    // Store previous state for calculating delta
    private Translation2d previousTranslation = new Translation2d(0, 0);
    private double previousRotation = 0;

    // Timestamp of last filter call (seconds)
    private double lastTime = -1;

    // Toggle filter on/off
    private boolean enabled = true;

    /**
     * Constructs a RateLimiterFilter.
     *
     * @param maxAccel maximum translational acceleration (m/s^2)
     * @param maxAngularAccel maximum rotational acceleration (rad/s^2)
     */
    public RateLimiterFilter(double maxAccel, double maxAngularAccel) {
        this.maxAccel = maxAccel;
        this.maxAngularAccel = maxAngularAccel;
    }

    /**
     * Toggle the filter on or off.
     * If disabled, the raw input passes through without limitation.
     *
     * @param enabled true to enable, false to disable
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Update the rate limits dynamically (optional for driver profiles).
     *
     * @param maxAccel maximum translational acceleration
     * @param maxAngularAccel maximum rotational acceleration
     */
    public void updateLimits(double maxAccel, double maxAngularAccel) {
        this.maxAccel = maxAccel;
        this.maxAngularAccel = maxAngularAccel;
    }

    /**
     * Apply rate limiting to input.
     *
     * @param inputTranslation joystick translational vector
     * @param inputRotation joystick rotation (rad/s)
     * @return Filtered InputState with limited rate of change
     */
    @Override
    public InputState filter(Translation2d inputTranslation, double inputRotation) {
        double currentTime = System.currentTimeMillis() / 1000.0;

        if (lastTime < 0) lastTime = currentTime; // first call
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        if (!enabled) {
            // Filter disabled, pass input through
            previousTranslation = inputTranslation;
            previousRotation = inputRotation;
            return new InputState(inputTranslation, inputRotation);
        }

        // --- Translational rate limiting ---
        Translation2d deltaTranslation = inputTranslation.minus(previousTranslation);
        double deltaMag = deltaTranslation.getNorm();

        double maxDelta = maxAccel * dt;
        if (deltaMag > maxDelta) {
            // Scale down delta vector to max allowed
            deltaTranslation = deltaTranslation.times(maxDelta / deltaMag);
        }
        Translation2d limitedTranslation = previousTranslation.plus(deltaTranslation);

        // --- Rotational rate limiting ---
        double deltaRotation = inputRotation - previousRotation;
        double maxDeltaRot = maxAngularAccel * dt;
        if (Math.abs(deltaRotation) > maxDeltaRot) {
            deltaRotation = Math.signum(deltaRotation) * maxDeltaRot;
        }
        double limitedRotation = previousRotation + deltaRotation;

        // Store for next call
        previousTranslation = limitedTranslation;
        previousRotation = limitedRotation;

        return new InputState(limitedTranslation, limitedRotation);
    }

    // TODO: Consider adding separate X/Y axis limits for fine driver control
    // TODO: Add logging for sudden input spikes to help with tuning
    // TODO: Allow different driver profiles to dynamically change limits
    // TODO: Potential optimization: handle dt = 0 or negative gracefully
}
