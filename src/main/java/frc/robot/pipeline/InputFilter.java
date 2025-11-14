package frc.robot.pipeline;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * InputFilter
 * 
 * Contract for all filters in the pipeline.
 * Filters take raw joystick translation + rotation and return a filtered InputState.
 * 
 * Every filter must obey:
 *  - Do not mutate the original inputs
 *  - Return a brand-new InputState
 *  - Handle vectors correctly (no cheating by separating X/Y unless documented)
 *  - Maintain deterministic, side-effect-free behavior
 */
public interface InputFilter {

    /**
     * Apply this filter to the input.
     *
     * @param inputTranslation translation vector from joystick (m/s)
     * @param inputRotation rotation input from joystick (rad/s)
     * @return filtered InputState
     */
    InputState filter(Translation2d inputTranslation, double inputRotation);
}
