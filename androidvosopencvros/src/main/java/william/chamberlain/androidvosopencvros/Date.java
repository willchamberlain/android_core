package william.chamberlain.androidvosopencvros;

import java.util.Calendar;
import android.support.annotation.NonNull;

import org.ros.message.Time;

import static java.util.Calendar.SECOND;

/**
 * Created by will on 24/02/17.
 */

public class Date {
    @NonNull
    public static org.ros.message.Time nowAsTime() {
        return org.ros.message.Time.fromMillis((new java.util.Date()).getTime());
    }

    public static java.util.Date toDate(org.ros.message.Time rosTime) {
//        long longNanoSeconds = (long)rosTime.nsecs;
//        long longSeconds     = (long)rosTime.secs;
//        long timeInMilliseconds = longNanoSeconds*1000L*1000L + longSeconds/1000L;
//        return new java.util.Date(timeInMilliseconds);
//        System.out.println(toDate);

        long timeInMilliseconds = rosTime.totalNsecs() / (1000L*1000L);
        System.out.println("toDate: rosTime[ s : ns ] = [ "+rosTime.toString()+" ] timeInMilliseconds="+timeInMilliseconds);
        return new java.util.Date( timeInMilliseconds );
    }

    public static org.ros.message.Time toRosTime(java.util.Date javaUtilDate_) {
        Calendar calendar = Calendar.getInstance();
        calendar.setTime(javaUtilDate_);
        int seconds      = calendar.get(SECOND);
        int milliseconds = calendar.get(Calendar.MILLISECOND);
        int nanoseconds  = milliseconds*1000*1000;
        return new org.ros.message.Time(seconds,nanoseconds);
    }
}
