package william.chamberlain.androidvosopencvros;

/**
 * Created by will on 2/03/17.
 */

public interface VariableResolution {
    void lowResolution();
    void highResolution();
    void resolutionMin(int width, int height);
    void resolutionMax(int width, int height);
    void resolutionMinMax(int minWidth, int minHeight,int maxWidth, int maxHeight);
}
