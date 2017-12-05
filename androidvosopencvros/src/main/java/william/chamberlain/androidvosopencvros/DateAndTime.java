package william.chamberlain.androidvosopencvros;

import java.util.Calendar;
import android.support.annotation.NonNull;

import org.ros.message.Time;
import org.ros.time.TimeProvider;

import static java.util.Calendar.SECOND;
import static william.chamberlain.androidvosopencvros.ProviderType.JAVA_UTIL_DATE;
import static william.chamberlain.androidvosopencvros.ProviderType.ROSJAVA;

/**
 * Created by will on 24/02/17.
 */

enum ProviderType{ROSJAVA, JAVA_UTIL_DATE}

public class DateAndTime {


    private static ProviderType providerType = JAVA_UTIL_DATE;
    private static final Object providerSyncObject = new Object();

    private static TimeProvider timeProvider = null;


    public static void configProviderToDefault() {
        synchronized (providerSyncObject) {
            providerType = JAVA_UTIL_DATE;
            System.out.println("Date.configProviderToDefault()");
        }
    }


    public static void configTimeProvider(TimeProvider timeProvider_) {
        synchronized (providerSyncObject) {
            timeProvider = timeProvider_;
            providerType = ROSJAVA;
            System.out.println("Date.configTimeProvider(" + timeProvider_ + ")");
        }
    }


    @NonNull
    public static org.ros.message.Time nowAsTime() {
        if(JAVA_UTIL_DATE == providerType) {
            System.out.println("Date.nowAsTime(): JAVA_UTIL_DATE == providerType");
            return org.ros.message.Time.fromMillis((new java.util.Date()).getTime());
        } else if(ROSJAVA == providerType) {
            System.out.println("Date.nowAsTime(): ROSJAVA == providerType");
            return timeProvider.getCurrentTime();
        } else {
            System.out.println("Date.nowAsTime(): exception: No time provider configured");
            throw new RuntimeException("No time provider configured");
        }
    }

    public static java.util.Date nowAsDate() {
        if(JAVA_UTIL_DATE == providerType) {
            System.out.println("Date.nowAsDate(): JAVA_UTIL_DATE == providerType");
            return new java.util.Date();
        } else if(ROSJAVA == providerType) {
            System.out.println("Date.nowAsDate(): ROSJAVA == providerType");
            return toDate(timeProvider.getCurrentTime());
        } else {
            System.out.println("Date.nowAsDate(): exception: No time provider configured");
            throw new RuntimeException("No time provider configured");
        }
    }

    /**
     * Returns date1 - date2 in milliseconds.
     * @param date1
     * @param date2
     * @return difference in milliseconds as date1 - date2.
     */
    public static long diffMs(java.util.Date date1, java.util.Date date2) {
        return date1.getTime() - date2.getTime();
    }

    /**
     * Returns date1 - date2 in nanoseconds.
     * @param date1
     * @param date2
     * @return difference in nanoseconds as date1 - date2.
     */
    public static long diffNs(java.util.Date date1, java.util.Date date2) {
        return diffMs(date1,date2)*(1000L*1000L);
    }

    public static java.util.Date toDate(org.ros.message.Time rosTime) {
        long timeInMilliseconds = rosTime.totalNsecs() / (1000L*1000L);
        System.out.println("Date.toDate: rosTime[ s : ns ] = [ "+rosTime.toString()+" ] timeInMilliseconds="+timeInMilliseconds);
        return new java.util.Date( timeInMilliseconds );
    }

    public static org.ros.message.Time toRosTime(java.util.Date javaUtilDate_) {
        Calendar calendar = Calendar.getInstance();
        calendar.setTime(javaUtilDate_);
        int seconds      = calendar.get(SECOND);
        int milliseconds = calendar.get(Calendar.MILLISECOND);
        int nanoseconds  = milliseconds*1000*1000;
        System.out.println("Date.toRosTime: rosTime[ s : ns ] = [ "+seconds+" : "+nanoseconds+" ] timeInMilliseconds="+milliseconds);
        return new org.ros.message.Time(seconds,nanoseconds);
    }
}
