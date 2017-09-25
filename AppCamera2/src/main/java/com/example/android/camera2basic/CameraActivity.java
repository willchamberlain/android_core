/*
 * Copyright 2014 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.android.camera2basic;

import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.util.Log;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;
import java.util.ArrayList;

import geometry_msgs.Pose;
import william.chamberlain.androidvosopencvros.DetectedFeaturesClient;
import william.chamberlain.androidvosopencvros.Hardcoding;
import william.chamberlain.androidvosopencvros.LocaliseFromAFeatureClient;
import william.chamberlain.androidvosopencvros.Naming;
import william.chamberlain.androidvosopencvros.PosedEntity;
import william.chamberlain.androidvosopencvros.RegisterVisionSourceClient;
import william.chamberlain.androidvosopencvros.RosThingy;
import william.chamberlain.androidvosopencvros.VariableResolution;
import william.chamberlain.androidvosopencvros.VisionSource;
import william.chamberlain.androidvosopencvros.VisionSourceManagementListener;
import william.chamberlain.androidvosopencvros.VosTaskSet;
import william.chamberlain.androidvosopencvros.device.DimmableScreen;

public class CameraActivity
        extends RosActivity
        implements PosedEntity                                    // Camera has a pose in the world; defaults to aligned with the map coordinate frame origin and axes.
        , DimmableScreen, VariableResolution, VisionSource
        , RosThingy
{
//        extends Activity {


    /*** implement DimmableScreen *****************************************************************/
    boolean displayRgb;

    @Override
    public void screenOff() {}

    @Override
    public void screenOn() {screenOn(100f);}

    @Override
    public void screenOn(float percentBrightness) {}

    @Override
    public void displayGrey() { displayRgb = false; }

    @Override
    public void displayRgb() { displayRgb = true; }

    /*** end implement DimmableScreen *****************************************************************/


    /*** implement VariableResolution *****************************************************************/
    @Override
    public void lowResolution() {}

    @Override
    public void highResolution() {}

    @Override
    public void resolutionMin(int width, int height) {}

    @Override
    public void resolutionMax(int width, int height) {}

    @Override
    public void resolutionMinMax(int minWidth, int minHeight, int maxWidth, int maxHeight) {}

    /*** end implement VariableResolution *****************************************************************/


    /*** end implement VisionSource *****************************************************************/
    @Override
    public void start() {}

    @Override
    public void stop() {}


    @Override
    public void startObstacleDetection(){}

    @Override
    public void stopObstacleDetection(){}

    @Override
    public void relocalise() {}

    @Override
    public void publishCurrentFrame() {}

    @Override
    public void allocateTo(String targetKey) {}

    /*** end implement VisionSource *****************************************************************/

    VosTaskSet vosTaskSet;

    private DetectedFeaturesClient detectedFeaturesClient;
    private VisionSourceManagementListener visionSourceManagementListener;
    private LocaliseFromAFeatureClient localiseFromAFeatureClient;
    private RegisterVisionSourceClient registerVisionSourceClient;

    public CameraActivity() {
        super("VOS", "VOS smart camera");
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) { // configure nodes: config gets fed to an AsyncTask to start the Nodes in a Bound Service: see https://developer.android.com/reference/android/app/Service.html , https://developer.android.com/guide/components/processes-and-threads.html
        vosTaskSet = new VosTaskSet();

        final String NODE_NAMESPACE = Naming.cameraNamespace(getCamNum());

        Log.i("CameraActivity","init: camNum="+getCamNum()+", NODE_NAMESPACE = "+NODE_NAMESPACE);
        URI masterURI = getMasterUri();
        Log.i("CameraActivity","init: masterURI='"+masterURI+"'");
        int currentapiVersion = android.os.Build.VERSION.SDK_INT;

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration8.setMasterUri(masterURI);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"detectedfeatures_serviceclient_node");
            this.detectedFeaturesClient = new DetectedFeaturesClient();
            detectedFeaturesClient.setCameraFrameId(Naming.cameraFrameId(getCamNum()));
            detectedFeaturesClient.setPosedEntity(this);
            nodeMainExecutor.execute(this.detectedFeaturesClient, nodeConfiguration8);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration.setMasterUri(masterURI);
            nodeConfiguration.setNodeName(NODE_NAMESPACE+"vision_source_management_topic_listener");
            this.visionSourceManagementListener = new VisionSourceManagementListener();
            visionSourceManagementListener.setNodeNamespace(NODE_NAMESPACE);
            visionSourceManagementListener.setDimmableScreen(this);
            visionSourceManagementListener.setVariableResolution(this);
            visionSourceManagementListener.setVisionSource(this);
            nodeMainExecutor.execute(this.visionSourceManagementListener, nodeConfiguration);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration8.setMasterUri(masterURI);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"localiseFromAFeature_serviceclient_node");
            this.localiseFromAFeatureClient = new LocaliseFromAFeatureClient();
            localiseFromAFeatureClient.setCameraFrameId(Naming.cameraFrameId(getCamNum()));
            localiseFromAFeatureClient.setPosedEntity(this);
            nodeMainExecutor.execute(this.localiseFromAFeatureClient, nodeConfiguration8);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration8.setMasterUri(masterURI);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"registervisionsource_serviceclient_node");
            this.registerVisionSourceClient = new RegisterVisionSourceClient();
            registerVisionSourceClient.setBaseUrl(Naming.cameraNamespace(getCamNum()));
            // TODO - do not set up the subscriber - having this setup in a close loop with creating the service does not work cleanly
            // TODO - could set the Subscribers up on demand - just frame as network config
            // TODO - this.whereIsSubscriber = new WhereIsSubscriber(this);
            // TODO - registerVisionSourceClient.setWhereIsSubscriber(whereIsSubscriber);
            Hardcoding.hardcodeTargetMarkers(vosTaskSet);
            nodeMainExecutor.execute(this.registerVisionSourceClient, nodeConfiguration8);
        }

    }

    private final int PERMISSIONS_REQUEST_ALL_AT_ONCE = 9001;
    private int numPermissionsRequested=0;


    private Camera2BasicFragment camera2BasicFragment;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);

        requestAllPermissions();

        camera2BasicFragment = Camera2BasicFragment.newInstance();
        camera2BasicFragment.rosThingy(this);

        if (null == savedInstanceState) {
            getFragmentManager().beginTransaction()
                    .replace(R.id.container, camera2BasicFragment)
                    .commit();
        }
    }

    @Override
    protected void onResume() {
        Log.i("CameraActivity","onResume: start");
        super.onResume();
        Log.i("CameraActivity","onResume: end");
    }


    /*****************************************/



    boolean registeredAsVisionSource = false;
    public void registerAsVisionSource() {                //// TODO - move into control loop
        if (!registeredAsVisionSource & null != registerVisionSourceClient) {
            String logTag = "c"+getCamNum();
            Log.i("CameraActivity",logTag+": registerAsVisionSource: start registering as a vision source");       //// TODO - timing here  c[camera_num]-f[frameprocessed]
            registerVisionSourceClient.registerVisionSource();
            registeredAsVisionSource = true;
            Log.i("CameraActivity",logTag+": registerAsVisionSource: finished registering as a vision source");    //// TODO - timing here  c[camera_num]-f[frameprocessed]
        }
    }

    public void reportDetectedFeature(int tagId, double x,double y,double z,double qx,double qy,double qz,double qw) {
        if(null == detectedFeaturesClient ) {
            return;
        }
        detectedFeaturesClient.reportDetectedFeature(tagId,
                x, y, z,
                qx, qy, qz, qw);
    }


    public void updateLocationFromDetectedFeature(int tagId, double x, double y, double z, double qx, double qy, double qz, double qw) {
        if(null == localiseFromAFeatureClient) {
            return;
        }
        String logTag = "c"+getCamNum();
        if (!poseKnown) {
            Log.i("CameraActivity",logTag+": updateLocationFromDetectedFeature: before localiseFromAFeatureClient.localiseFromAFeature");
            localiseFromAFeatureClient.localiseFromAFeature(tagId,
                    x, y, z,
                    qx, qy, qz, qw);
            //// TODO - timing here  c[camera_num]-f[frameprocessed]-i[iteration]-t[tagid]
            Log.i("CameraActivity",logTag+": updateLocationFromDetectedFeature: after localiseFromAFeatureClient.localiseFromAFeature");
        }
    }


    /******************************************/

    private boolean requestAllPermissions() {
        String[] allMissingPermissionsAtOnce = new String[]{};
        numPermissionsRequested=0;
        ArrayList<String> buildAllMissingPermissionsAtOnce = new ArrayList<String>(0);
        checkPermissionAndBuildSet(buildAllMissingPermissionsAtOnce, Manifest.permission.ACCESS_WIFI_STATE);
        checkPermissionAndBuildSet(buildAllMissingPermissionsAtOnce, Manifest.permission.CHANGE_WIFI_STATE);
        checkPermissionAndBuildSet(buildAllMissingPermissionsAtOnce, Manifest.permission.INTERNET);
        checkPermissionAndBuildSet(buildAllMissingPermissionsAtOnce, Manifest.permission.WAKE_LOCK);
        checkPermissionAndBuildSet(buildAllMissingPermissionsAtOnce, Manifest.permission.WRITE_EXTERNAL_STORAGE);
        checkPermissionAndBuildSet(buildAllMissingPermissionsAtOnce, Manifest.permission.ACCESS_FINE_LOCATION);
        checkPermissionAndBuildSet(buildAllMissingPermissionsAtOnce, Manifest.permission.NFC);
        checkPermissionAndBuildSet(buildAllMissingPermissionsAtOnce, Manifest.permission.CAMERA);
        if (buildAllMissingPermissionsAtOnce.size() > 0) {
            allMissingPermissionsAtOnce = buildAllMissingPermissionsAtOnce.toArray(allMissingPermissionsAtOnce);
            requestPermissions(allMissingPermissionsAtOnce, PERMISSIONS_REQUEST_ALL_AT_ONCE);
            return false;
        } else {
            return true;
        }
    }

    private void checkPermissionAndBuildSet(ArrayList<String> buildAllMissingPermissionsAtOnce, String permissionName) {
        if (checkSelfPermission(permissionName) != PackageManager.PERMISSION_GRANTED) {
            Log.i("CameraActivity","checkPermissionAndBuildSet: need to request permission "+permissionName);
            buildAllMissingPermissionsAtOnce.add(permissionName);
            numPermissionsRequested++;
        } else {
            Log.i("CameraActivity","checkPermissionAndBuildSet: already have permission "+permissionName);
        }
    }

    private boolean allGranted(int[] grantResults) {
        for (int grantResult:
                grantResults) {
            if (grantResult != PackageManager.PERMISSION_GRANTED) {
                Log.w("CameraActivity","allGranted: "+grantResult+" != "+PackageManager.PERMISSION_GRANTED);
                return false;
            }
        }
        return true;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        switch (requestCode) {
            case PERMISSIONS_REQUEST_ALL_AT_ONCE:
                if (grantResults.length > 0
                        && grantResults.length >= numPermissionsRequested
                        && allGranted(grantResults)) { //permission granted successfully
                    Log.i("CameraActivity","onRequestPermissionsResult: got all permissions: no action here");

                } else { //permission denied
                    Log.i("CameraActivity","onRequestPermissionsResult: did not get all permissions: finishing: grantResults.length="+grantResults.length+", numPermissionsRequested="+numPermissionsRequested);
                    finish();
                }
                break;
            default:
                Log.i("CameraActivity","onRequestPermissionsResult: requestCode "+requestCode+" is not recognised.");
                break;
        }
    }
    /******************************************/



    /*** implements PosedEntity ***************************************/

    private double[] position    = {0.0,0.0,1.0};     // = new double[3]
    private double[] orientation = {0.0,0.0,0.0,1.0}; // = new double[4]
    private boolean  poseKnown   = false;

    @Override
    public void setPose(double[] poseXyz, double[] orientationQuaternionXyzw_) {
        this.position = poseXyz;
        this.orientation = orientationQuaternionXyzw_;
        this.poseKnown = true;
    }

    public void setPose(Pose pose_) {
        setPose(
                new double[]{pose_.getPosition().getX(),pose_.getPosition().getY(),pose_.getPosition().getZ()},
                new double[]{pose_.getOrientation().getX(),pose_.getOrientation().getY(),pose_.getOrientation().getZ(),pose_.getOrientation().getW()});
    }

    public double[] getPositionXyz() {
        return position;
    }

    public double[] getOrientationQuaternionXyzw() {
        return orientation;
    }

    public boolean poseKnown(){
        return poseKnown;
    }

    /*** end implements PosedEntity ***************************************/

}
