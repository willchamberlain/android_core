package william.chamberlain.androidvosopencvros;

/**
 * Created by will on 3/03/17.
 */

public interface VisionSource {
    void start();
    void stop();
    void relocalise();
    void publishCurrentFrame();
    void allocateTo(String targetKey);

    public static final String ROBOT_ALLOCATION_KEY = "ROBOT";
    public static final String TARGET_ALLOCATION_KEY = "TARGET";
}
