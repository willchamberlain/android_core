package william.chamberlain.androidvosopencvros;

/**
 * Created by will on 13/04/17.
 */

public interface AlgorithmParameters {
    int binaryThreshold();
    boolean parametersHaveChanged();
    boolean parametersHaveChanged(boolean newValue);
}
