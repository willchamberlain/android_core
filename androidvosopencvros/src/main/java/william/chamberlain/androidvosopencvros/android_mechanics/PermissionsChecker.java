package william.chamberlain.androidvosopencvros.android_mechanics;

import android.app.Activity;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.os.Build;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.widget.Toast;

/**
 *
 * For the standard, see https://developer.android.com/training/permissions/requesting.html
 * , https://developer.android.com/training/permissions/declaring.html
 * , https://developer.android.com/training/permissions/usage-notes.html .
 *
 * For discussion see https://stackoverflow.com/questions/32347532/android-m-permissions-confused-on-the-usage-of-shouldshowrequestpermissionrati
 * For explanation see https://inthecheesefactory.com/blog/things-you-need-to-know-about-android-m-permission-developer-edition/en
 *
 * For Google's solution to the basic case, see https://github.com/googlesamples/easypermissions
 *
 * Current code from https://stackoverflow.com/questions/20666366/it-seems-that-your-device-does-not-support-cameraor-it-is-locked
 *
 * Created by will on 25/05/17.
 */

public class PermissionsChecker {

    final static public int REQUEST_CODE_ASK_PERMISSIONS = 123;

    /** activityThatWantsPermission e.g.: MainActivity.this . */
    final private Activity activityThatWantsPermission;


    /**
     * @param activityThatWantsPermission_ e.g.: MainActivity.this .
     */
    public PermissionsChecker(Activity activityThatWantsPermission_) {
        activityThatWantsPermission = activityThatWantsPermission_;
    }

    /**
     *
     * @param activityThatWantsPermission e.g.: MainActivity.this .
     * @param permission one of the android.permission permission Strings.
     * @return true if currently have the permission, false otherwise.
     */
    public boolean havePermission(Activity activityThatWantsPermission, String permission) {
        int currentapiVersion = android.os.Build.VERSION.SDK_INT;
        // First check android version
        if (currentapiVersion > Build.VERSION_CODES.LOLLIPOP_MR1) {
            //Check if permission is already granted
            if (ContextCompat.checkSelfPermission(activityThatWantsPermission,
                    permission)
                    != PackageManager.PERMISSION_GRANTED) {
                return false;
            }
        }
        return true;
    }

    /**
     * @param permission one of the android.permission permission Strings.
     * @param message e.g.  "You need to allow access to the camera" , "You need to allow access to Contacts"
     * @return true if currently have the permission, false otherwise: note that any user interaction dialogs that may enable the permission are dealt with asynchronously.
     */
    public boolean havePermissionAndRequest(final String permission, final String message) {
        int currentapiVersion = android.os.Build.VERSION.SDK_INT;
        // First check android version
        if (currentapiVersion > Build.VERSION_CODES.LOLLIPOP_MR1) {
            //Check if permission is already granted
            if (ContextCompat.checkSelfPermission(activityThatWantsPermission,
                    permission)
                    != PackageManager.PERMISSION_GRANTED) {

                // Give first an explanation, if needed.
                if (ActivityCompat.shouldShowRequestPermissionRationale(activityThatWantsPermission,
                        permission)) {

                    // Show an explanation to the user *asynchronously* -- don't block
                    // this thread waiting for the user's response! After the user
                    // sees the explanation, try again to request the permission.

                    // TODO - just informs the user, doesn't give them option to enable permission through the dialog
//                    Dialog.showMessageOK(activityThatWantsPermission, message);

                    //  TODO  !!  -  probably needs to be in a separate module that is only built for API level 23+, which introduces Activity.requestPermissions
                    Dialog.showMessageOKCancel(
                            activityThatWantsPermission,
                            permission,
                            message,
                            new DialogInterface.OnClickListener() {
                                @Override
                                public void onClick(DialogInterface dialog, int which) {
                                    ActivityCompat.requestPermissions(
                                            activityThatWantsPermission,
                                            new String[] {permission},
                                            REQUEST_CODE_ASK_PERMISSIONS);
                                }
                            });
                    //  end TODO  !!  -  probably needs to be in a separate module that is only built for API level 23+, which introduces Activity.requestPermissions

                } else {

                    // No explanation needed, we can request the permission.

                    ActivityCompat.requestPermissions(activityThatWantsPermission,
                            new String[]{permission},
                            REQUEST_CODE_ASK_PERMISSIONS);
                }
                return false;
            }
        }
        return true;
    }



}
