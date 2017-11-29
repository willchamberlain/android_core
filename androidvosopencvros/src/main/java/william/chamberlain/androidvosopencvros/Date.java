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

    public static java.util.Date toDate(Time rosTime) {
        long longNanoSeconds = (long)rosTime.nsecs;
        long longSeconds     = (long)rosTime.secs;
        long timeInMilliseconds = longNanoSeconds*1000L*1000L + longSeconds/1000L;
        return new java.util.Date(timeInMilliseconds);
    }
}
