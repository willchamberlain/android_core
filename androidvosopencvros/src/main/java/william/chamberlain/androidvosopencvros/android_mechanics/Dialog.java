package william.chamberlain.androidvosopencvros.android_mechanics;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;

public class Dialog {
    public static void showMessageOKCancel(Activity activityThatWantsPermission, String permission, String message, DialogInterface.OnClickListener okListener) {
        new AlertDialog.Builder(activityThatWantsPermission)
                .setMessage(message)
                .setPositiveButton("OK", okListener)
                .setNegativeButton("Cancel", null)
                .create()
                .show();
    }

    public static void showMessageOK(Activity activityThatWantsPermission, String permission, String message) {
        new AlertDialog.Builder(activityThatWantsPermission)
                .setMessage(message)
                .setPositiveButton("OK", null)
                .create()
                .show();
    }
}