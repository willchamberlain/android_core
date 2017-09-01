package william.chamberlain.androidvosopencvros;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;

public class Hardcoding {


    public static final int MARKER_OFFSET_INT = 90000;
    public static final double BOOFCV_MARKER_SIZE_M = 0.257; //0.189;  // 0.20  // 0.14             ////  TODO - list of tags and sizes, and tag-groups and sizes


    public static void hardcodeTargetMarkers(VosTaskSet vosTaskSet) {
        // robot target marker
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "210", 1));
        // robot model markers
//        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "170", 1,  0.10,  0.00, 0.42,  0,    0,    0,          1 ));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "170", 1,  -0.05, 0.00, 0.79,  0.0d, 0.0d,  0.382696d, 0.923874d ));  // upper left  - 79 cm up, 45 deg left
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "250", 1,  -0.05, 0.00, 0.79,  0.0d, 0.0d, -0.382696d, 0.923874d ));  // upper right - 79 cm up, 45 deg right
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "290", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "330", 1));
        // fixed-position markers for PnP
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "650", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "690", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "730", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "770", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "810", 1));

        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "850", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "930", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1090", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1010", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "970", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1050", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "890", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1250", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1210", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1170", 1));

        /*** Quad for boofcv P3P/PnP  ***/
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "970", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1210", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1250", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1170", 1));


        /** check Nexus6 accuracy ***/
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1610", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1690", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1730", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1650", 1));

        /** big markers for distance ***/
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv",  "57", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "157", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "257", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "357", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "457", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "557", 1));
    }


    /* Dev: part of robot visual model */
    public static boolean isPartOfRobotVisualModel(final int tag_id) {
        return tag_id == 170 || tag_id == 250 || tag_id == 290 || tag_id == 330
                || tag_id == 1650
                || tag_id == 57 || tag_id == 157 || tag_id == 257 || tag_id == 357 || tag_id == 457 || tag_id == 557
                ;
    }

    /* Dev: part of knowledge about landmarks */
    public static boolean isALandmark(final int tag_id) {
        return tag_id == 650 || tag_id == 690 || tag_id == 730 || tag_id == 770 || tag_id == 810 || tag_id == 850 || tag_id == 930 || tag_id == 1090 || tag_id == 1010 || tag_id == 970 || tag_id == 1050 || tag_id == 890 || tag_id == 1250 || tag_id == 1210 || tag_id == 1170
                || tag_id == 1610 || tag_id == 1690 || tag_id == 1730 || tag_id == 170
                ;
    }

    public static boolean isAQuad(final int tag_id) {
        return tag_id == 970 || tag_id == 1210 || tag_id == 1250 || tag_id == 1170;
    }
}