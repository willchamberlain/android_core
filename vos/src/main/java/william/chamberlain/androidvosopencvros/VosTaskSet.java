package william.chamberlain.androidvosopencvros;

import android.util.Log;

import java.util.ArrayList;

import vos_aa1.WhereIsAsPub;

/**
 * Created by will on 7/08/17.
 */

public class VosTaskSet {


    ArrayList<VisionTask> taskQueue = new ArrayList<VisionTask>();

    public boolean isThereAVisionTaskToExecute(int tag_id, String logTagTag) {
        boolean visionTaskToExecute = false;
        synchronized (this) {
        for (VisionTask visionTask : taskQueue) {
//            Log.i(logTagTag,"visionTask.getDescriptor() = '"+visionTask.getDescriptor()+"', Integer.toString(tag_id) = '"+Integer.toString(tag_id)+"' ");
            if(visionTask.getDescriptor().equals(Integer.toString(tag_id))) {
//                Log.i(logTagTag,"will execute vision task "+visionTask);
                visionTaskToExecute = true;
            }
        }
        }
        return visionTaskToExecute;
    }

    public boolean isThereAVisionTaskToExecute() {
        synchronized (this) {
            return !taskQueue.isEmpty();
        }
    }

    ArrayList<VisionTask> taskQueue() {
        return this.taskQueue;
    }

    public void addVisionTaskToQueue(WhereIsAsPub message) {
        synchronized (this) {
            taskQueue.add(new VisionTask()
                    .algorithm(message.getAlgorithm())
                    .descriptor(message.getDescriptor())
                    .requestId(message.getRequestId())
                    .relationToBase(message.getRelationToBase())
                    .returnUrl(message.getReturnUrl())
                    .executionIterations(message.getRate()));
        }
    }

    public void addVisionTaskToQueue(WhereIsAsPubLocal message) {
        synchronized (this) {
            taskQueue.add(new VisionTask()
                    .algorithm(message.getAlgorithm())
                    .descriptor(message.getDescriptor())
                    .requestId(message.getRequestId())
                    .relationToBase(message.getRelationToBase())
                    .returnUrl(message.getReturnUrl())
                    .executionIterations(message.getRate()));
        }
    }


    public void removeExpiredVisionTasks() {
        ArrayList<VisionTask> toRemove = new ArrayList<VisionTask>();
        synchronized (this) {
            for (VisionTask task : taskQueue) {  // TODO - wrap this up in a VisionTaskQueue, and probably move to top or tail of the process , and look at e.g. ArrayBlockingQueue
                Log.i("VosTaskSet","removeExpiredVisionTask: vision task is now "+task);
                if(!task.canBeExecuted()) {
                    toRemove.add(task);     // could leave them in and only remove once a few have built up
                    Log.i("VosTaskSet","removeExpiredVisionTask: removed vision task "+task);
                }
                task.executed();
            }
            // TODO - not removing - for quicker test setup
            // TODO     taskQueue.removeAll(toRemove);
            // TODO - not removing - for quicker test setup
        }
    }
}
