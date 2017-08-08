package william.chamberlain.androidvosopencvros;

public class Hardcoding {

    public static void hardcodeTargetMarkers(VosTaskSet vosTaskSet) {
        // robot target marker
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "210", 1));
        // robot model markers
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "170", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "250", 1));
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
    }
}