package william.chamberlain.androidvosopencvros;

public enum ManagementCommand {
        SCREEN_ON("screen on"),
        SCREEN_OFF("screen off");

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