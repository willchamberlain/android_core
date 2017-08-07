package william.chamberlain.androidvosopencvros;

import java.util.HashMap;

public class LandmarkFeatureLoader {


    HashMap<Integer, double[]> loadLandmarkFeatures() {
        HashMap<Integer, double[]> landmarks = new HashMap<Integer, double[]>();
    /*  TODO - HARDCODING  */
        landmarks.put(690, new double[]{0.0, -40.0, 510.0});  // todo - some visualisation of the world points nominated - e.g. --> RViz markers
        landmarks.put(650, new double[]{0.0, 380.0, 510.0});
        landmarks.put(770, new double[]{0.0, 380.0, 0.0});
        landmarks.put(730, new double[]{0.0, 0.0, 0.0});
        landmarks.put(850, new double[]{0.0, 2560, 1540});
        landmarks.put(930, new double[]{0.0, 2270, 1170});
        landmarks.put(1090, new double[]{0.0, 1955, 1490});
        landmarks.put(1010, new double[]{0.0, 1650, 1540});
        landmarks.put(970, new double[]{0.0, 1365, 1070});
        landmarks.put(1050, new double[]{0.0, 1060, 1390});
        landmarks.put(890, new double[]{0.0, 1050, 1020});
        landmarks.put(1250, new double[]{-280, 0.0, 1340});
        landmarks.put(1210, new double[]{-380, 0.0, 920});
        landmarks.put(1170, new double[]{-750, 0.0, 1260});
    /*  TODO - HARDCODING  */
        return landmarks;
    }
}