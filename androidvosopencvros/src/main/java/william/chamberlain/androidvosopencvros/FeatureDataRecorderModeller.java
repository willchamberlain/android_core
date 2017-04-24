package william.chamberlain.androidvosopencvros;

import java.util.HashMap;

/**
 * Records the detection of features,
 * and smooths/filters the detections
 * - including correlating or fusing with other data streams -
 * to provide a coherent model of detections over time,
 * and specifically the most recent detections.
 */

public class FeatureDataRecorderModeller {
    public static int DEFAULT_MODEL_DATA_TRACK_SIZE = 20;
    HashMap<String, DataTrack> dataTracks = new HashMap<String, DataTrack>();



    public DataTrack addDetection(DetectedFeature detectedFeature){
        String id = detectedFeature.featureCanonicalDescriptor();
        DataTrack track;
        if (dataTracks.containsKey(id)) {
            track = dataTracks.get(id);
        } else {
            track = new DataTrack(DEFAULT_MODEL_DATA_TRACK_SIZE);
            dataTracks.put(id, track);
        }
        track.add(detectedFeature);
        return track;
    }

    private DetectedFeature averageData(DetectedFeature[] features_) {
        throw new RuntimeException("Implementation incomplete");
//        if(null==features_ || 0 == features_.length) {
//            return null;
//        }
//        for (DetectedFeature feature : features_) {
//
//        }
    }

    public DetectedFeature average(String algorithm_, String descriptor_) {
        return averageData(dataTracks.get(Descriptor.featureCanonicalDescriptor(algorithm_,descriptor_)).data());
    }

}


