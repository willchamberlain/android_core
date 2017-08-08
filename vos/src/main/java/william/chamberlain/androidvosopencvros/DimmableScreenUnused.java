package william.chamberlain.androidvosopencvros;

/**
 * Instances have a screen that can be turned off / dimmed as far as possible, turned on to as bright as possible, or turned on to a percentage of the maximum brightness.
 *
 * Created by will on 2/03/17.
 */
public interface DimmableScreenUnused {

    /**
     * Turns off / dims the screen as far as possible.
     */
    void screenOff();

    /**
     * Turns on the screen to as bright as possible.
     */
    void screenOn();

    /**
     * Turns on the screen to a percentage of the maximum brightness: <code>percentBrightness</code> is in the range 0.0 (0%: as dim as possible) to 1.0 (100%; as bright as possible).
     *
     * @param percentBrightness brightness to set in the range 0.0 (0%: as dim as possible) to 1.0 (100%; as bright as possible). Values less than 0.0 are treated as 0.0. Values greater than 1.0 are treated as 1.0.
     */
    void screenOn(float percentBrightness);
}
