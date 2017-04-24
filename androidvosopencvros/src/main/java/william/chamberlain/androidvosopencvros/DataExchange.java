package william.chamberlain.androidvosopencvros;

import java.util.regex.Pattern;

/**
 * Created by will on 24/04/17.
 */

public class DataExchange {
    public static final Pattern tagPattern_trans_quat = Pattern.compile("tag ([0-9]+) at x=([0-9-]+\\.[0-9]+) y=([0-9-]+\\.[0-9]+) z=([0-9-]+\\.[0-9]+) qx=([0-9-]+\\.[0-9]+) qy=([0-9-]+\\.[0-9]+) qz=([0-9-]+\\.[0-9]+) qw=([0-9-]+\\.[0-9]+)");
    public static final Pattern dataOutputPattern = Pattern.compile("data ([A-z0-9_]+)\\:([A-z0-9_\\.]+)");
}
