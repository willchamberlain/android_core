package william.chamberlain.androidvosopencvros;

import java.util.Calendar;
import android.support.annotation.NonNull;

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
            org.ros.message.Time rosTime = timeProvider.getCurrentTime();
            java.util.Date       date    = toJavaDate(rosTime);
            System.out.println("Date.nowAsDate(): ROSJAVA == providerType: date="+date.getTime()+"ms: rosTime="+(rosTime.totalNsecs()/(1000L*1000L))+"ms="+rosTime.totalNsecs()+"ns");
            return date;
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
    public static long diffMs(java.util.Date date1, java.util.Date date2) { return date1.getTime() - date2.getTime(); }
    /**
     * Returns date1 - date2 in nanoseconds.
     * @param date1
     * @param date2
     * @return difference in nanoseconds as date1 - date2.
     */
    public static long diffNs(java.util.Date date1, java.util.Date date2) { return diffMs(date1,date2)*(1000L*1000L); }

    public static long diffNs(java.util.Date java_date1, org.ros.message.Time ros_time2) { return java_date1.getTime()*(1000L*1000L) - ros_time2.totalNsecs(); }
    public static long diffNs(org.ros.message.Time ros_time1, java.util.Date java_date2) { return diffNs(java_date2, ros_time1); }
    public static long diffMs(org.ros.message.Time ros_time1, java.util.Date java_date2) { return diffNs(java_date2, ros_time1)/(1000*1000); }
    public static long diffMs(java.util.Date java_date1, org.ros.message.Time ros_time2) { return diffNs(java_date1, ros_time2)/(1000*1000); }

    public static long diffMs(org.ros.message.Time ros_time1, org.ros.message.Time ros_time2) { return diffNs(ros_time1,ros_time2)/(1000*1000); }
    public static long diffNs(org.ros.message.Time ros_time1, org.ros.message.Time ros_time2) { return ros_time1.totalNsecs() - ros_time2.totalNsecs(); }

    public static java.util.Date toJavaDate(org.ros.message.Time rosTime) {
        long timeInMilliseconds = rosTime.totalNsecs() / (1000L*1000L);
        System.out.println("Date.toJavaDate: rosTime[ s : ns ] = [ "+rosTime.toString()+" ] timeInMilliseconds="+timeInMilliseconds);
        return new java.util.Date( timeInMilliseconds );
    }

    public static int safeLongToInt(long l) {
        int i = (int)l;
        if ((long)i != l) {
            throw new IllegalArgumentException(l + " cannot be cast to int without changing its value.");
        }
        return i;
    }

    public static org.ros.message.Time toRosTime(java.util.Date javaUtilDate_) {
        Calendar calendar = Calendar.getInstance();
        calendar.setTime(javaUtilDate_);
        long millisecondsInDate     = javaUtilDate_.getTime();
        long secondsInDate          = millisecondsInDate / (1000L);
        int  secondsInDateInt       = safeLongToInt(secondsInDate);
        long justMillisecondsInDate = millisecondsInDate - (secondsInDate*1000L);
        long nanosecondsInDate      = (justMillisecondsInDate) * (1000L*1000L);
        int  nanosecondsInDateInt   = safeLongToInt(nanosecondsInDate);

        int seconds      = calendar.get(SECOND);
        int milliseconds = calendar.get(Calendar.MILLISECOND);
        int nanoseconds  = milliseconds*1000*1000;
        System.out.println("Date.toRosTime: rosTime[ s : ns ] = [ "+seconds+" : "+nanoseconds+" ]: timeInMilliseconds="+milliseconds);
//        return new org.ros.message.Time(seconds,nanoseconds);

        org.ros.message.Time rosTime = new org.ros.message.Time(secondsInDateInt,nanosecondsInDateInt);

        System.out.println("Date.toRosTime: " +
                "millisecondsInDate="+millisecondsInDate+": " +
                "secondsInDate long="+secondsInDate+": " +
                "secondsInDateInt="+secondsInDateInt+": " +
                "justMillisecondsInDate="+justMillisecondsInDate+": " +
                "nanosecondsInDate="+nanosecondsInDate+": " +
                "nanosecondsInDateInt="+nanosecondsInDateInt+": " +
                "rosTime="+rosTime);
        return rosTime;
    }

}
