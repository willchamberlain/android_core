package william.chamberlain.androidvosopencvros;


import actionlib_msgs.GoalStatus;
import geometry_msgs.PoseWithCovarianceStamped;
import georegression.struct.se.Se3_F64;

import org.ros.rosjava_geometry.Transform;

import william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator;

public class Test_SmartCameraExtrinsicsCalibrator {

    class MockItAll implements RobotPoseMeasure, RobotGoalPublisher, RobotStatusMonitor {

        /*** RobotPoseMeasure ******************************************************/
        public Transform askRobotForPose() {
            return null;
        }


        /*** RobotGoalPublisher ******************************************************/

        public void sendRobotGoalInWorldFrame(PoseStamped poseStamped_) {

        }

        public void sendRobotGoalInWorldFrame(Transform transform_) {

        }

        public void sendRobotGoalInRobotFrame(Transform transform_) {

        }


        /*** RobotStatusMonitor ******************************************************/

        public GoalStatus[] robotStatusChange() {
            return null;
        }

        public void addRobotStatusChangeListener(RobotStatusChangeListener listener_) {

        }

        public void removeRobotStatusChangeListener(RobotStatusChangeListener listener_) {

        }



    }

    public static void main(String[] args) {
        System.out.println("I can see my house from here.");
        SmartCameraExtrinsicsCalibrator extrinsicsCalibrator = new SmartCameraExtrinsicsCalibrator();
        MockItAll bob = new MockItAll();
        extrinsicsCalibrator.setRobotPoseMeasure(bob);
        extrinsicsCalibrator.setRobotGoalPublisher(bob);
        bob.addRobotStatusChangeListener(extrinsicsCalibrator);
        System.out.println("Nice Garry");
    }

}