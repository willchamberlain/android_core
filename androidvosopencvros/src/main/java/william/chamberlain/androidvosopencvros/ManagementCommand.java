package william.chamberlain.androidvosopencvros;

public enum ManagementCommand {
        SCREEN_ON("screen on"),
        SCREEN_OFF("screen off"),
        PROCESSING_ON("run"),
        PROCESSING_OFF("stop"),
        RESOLUTION_HIGH("high res"),
        RESOLUTION_LOW("low res"),
        RESOLUTION_LIMITS("res limits"),
        RELOCALISE("relocalise"),
        DISPLAY_RGB("rgb"),
        DISPLAY_GREY("grey"),
        ALLOCATE_TO_ROBOT("task: robot 170 290 250 330"),
        ALLOCATE_TO_TARGET("task: target 210");

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