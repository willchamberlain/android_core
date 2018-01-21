package william.chamberlain.androidvosopencvros;

public enum ManagementCommand {
        SCREEN_ON("screen on"),
        SCREEN_ON_PREFERRED("screen preferred"),
        SCREEN_ON_BRIGHT("screen on bright"),
        SCREEN_ON_MID("screen on mid"),
        SCREEN_ON_LOW("screen on low"),
        SCREEN_OFF("screen off"),
        PROCESSING_ON("run"),
        PROCESSING_OFF("stop"),
        OBSTACLE_DETECTION_ON("run obstacle"),
        OBSTACLE_DETECTION_ON_HSV("run obstacle HSV"),
        OBSTACLE_DETECTION_ON_TEXTURE("run obstacle texture"),
        OBSTACLE_DETECTION_PROJECT("run obstacle project"),
        OBSTACLE_DETECTION_PROJECT_HIST("run obstacle history project"),
        OBSTACLE_DETECTION_OFF("stop obstacle"),
        RESOLUTION_VERY_VERY_HIGH("res very very high"),
        RESOLUTION_VERY_HIGH("res very high"),
        RESOLUTION_HIGH("res high"),
        RESOLUTION_LOW("res LOW"),
        RESOLUTION_LIMITS("res limits"),
        RELOCALISE("relocalise"),
        DISPLAY_RGB("rgb"),
        DISPLAY_GREY("grey"),
//        LOCATION("loc"),
//        ORIENTATION("orientation"),
//        POSE("pose"),
        ALLOCATE_TO_ROBOT("task: robot 170 290 250 330"),
        ALLOCATE_TO_TARGET("task: target 210"),
        EXTERNAL_CALIBRATION_CAPTURE_ONE_IMAGE("external calibration capture one image"),
        EXTERNAL_CALIBRATION_CAPTURE_STOP("external calibration stop capture"),
        EXTERNAL_CALIBRATION_CAPTURE_CONTINUOUS("external calibration continuous"),
        EXTERNAL_CALIBRATION_REQUEST_CURRENT_ROBOT_POSE("external calibration request current robot pose"),
        EXTERNAL_CALIBRATION_TAKE_OBSERVATION("external calibration take observation"),
        EXTERNAL_CALIBRATION_LIST_OBSERVATIONS("external calibration list observations"),
        EXTERNAL_CALIBRATION_RESET("external calibration reset"),
        RECORD_DATA_ONCE("record once"),
        RECORD_DATA_CONTINUOUS("record start"),
        RECORD_DATA_STOP("record stop"),
        RECORD_NEXT_FRAME("record next frame");

        public final String command_val;

        ManagementCommand(String theCommand) {
            command_val = theCommand;
        }

        public static ManagementCommand findByKey(String name) {
            ManagementCommand[] testEnums = ManagementCommand.values();
            for (ManagementCommand testEnum : testEnums) {
                if (testEnum.command_val.equals(name)) {
                    return testEnum;
                }
            }
            return null;
        }

}