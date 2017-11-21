package william.chamberlain.androidvosopencvros;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by will on 16/11/17.
 */

public class SmartCameraTopLevelController {
    List<SmartCameraBehaviour> currentBehaviours = new ArrayList<>(1);

    public void statusChanged(final int robot_id, final byte new_status) {
        for(SmartCameraBehaviour behaviour: currentBehaviours) {
            behaviour.statusChanged(robot_id, new_status);
        }
    }
}
