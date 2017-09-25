package william.chamberlain.androidvosopencvros;

/**
 * Created by will on 25/09/17.
 */

public enum Algorithm {
    SIFT("SIFT"),
    SURF("SURF"),
    APRIL_TAGS_KAESS_36_H_11("AprilTags_Kaess_36h11"),
    BOOFCV_SQUARE_FIDUCIAL("boofcv square fiducial"),
    FREE_SPACE("free space"),
    OBSTACLES("obstacles");

    private final String canonicalName;

    private Algorithm(String canonicalName_) {
        this.canonicalName = canonicalName_;
    }

    public String canonicalName() {
        return canonicalName;
    }

    public String toString() {
        return canonicalName;
    }
}
