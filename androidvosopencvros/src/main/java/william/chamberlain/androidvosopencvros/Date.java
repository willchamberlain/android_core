package william.chamberlain.androidvosopencvros;

import android.support.annotation.NonNull;

import org.ros.message.Time;

/**
 * Created by will on 24/02/17.
 */

public class Date {
    @NonNull
    public static Time nowAsTime() {
        return Time.fromMillis((new java.util.Date()).getTime());
    }
}
