package william.chamberlain.androidvosopencvros;

/**
 * Created by will on 27/10/17.
 */

public enum VisionTaskType {
    LOCALISE_CAMERA("LOCALISE_CAMERA"),
    LOCALISE_EXTERNAL("LOCALISE_EXTERNAL");



    public final String taskType;

    VisionTaskType(String taskType_) {
        taskType = taskType_;
    }

    public static VisionTaskType findByKey(String name) {
        VisionTaskType[] testEnums = VisionTaskType.values();
        for (VisionTaskType testEnum : testEnums) {
            if (testEnum.taskType.equals(name)) {
                return testEnum;
            }
        }
        return null;
    }
}
