package william.chamberlain.androidvosopencvros;

import android.support.annotation.NonNull;

public class Descriptor {

    @NonNull
    public static String featureCanonicalDescriptor(final DetectedFeature detectedFeature) {
        return featureCanonicalDescriptor(detectedFeature.algorithm, detectedFeature.descriptor);
    }

    @NonNull
    public static String featureCanonicalDescriptor(String algorithm_, String descriptor_) {
        return algorithm_ + "___" + descriptor_;
    }
}