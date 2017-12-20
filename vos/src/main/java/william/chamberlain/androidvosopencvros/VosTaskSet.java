package william.chamberlain.androidvosopencvros;

import android.util.Log;

import java.util.ArrayList;
import java.util.List;

import vos_aa1.WhereIsAsPub;

/**
 * Created by will on 7/08/17.
 */

public class VosTaskSet {


    ArrayList<VisionTask> taskQueue = new ArrayList<VisionTask>();


    //----------------------------------------------------------------------------------------------


    /**
     * VosTaskSet thingsIShouldBeLookingFor = ...
     * if(thingsIShouldBeLookingFor.includes("boofcv square fiducial")) ...
     *
     * @param algorithm_  currently arbitrary.
     * @return
     */
    public boolean includes(Algorithm algorithm_) {
        return isThereAVisionTaskToExecuteForAlgorithm(algorithm_);
    }


    private boolean isThereAVisionTaskToExecuteForAlgorithm(Algorithm algorithm_) {
        synchronized (this) {
            for (VisionTask visionTask : taskQueue) {
                if(visionTask.getAlgorithm().equals(algorithm_.canonicalName())) {
                    return true;
                }
            }
        }
        return false;
    }

    //----------------------------------------------------------------------------------------------

    public boolean includes(Algorithm algorithm_, int tag_id) {
        return isThereAVisionTaskToExecute(algorithm_, tag_id);
    }

    public boolean isThereAVisionTaskToExecute(Algorithm algorithm_, int tag_id) {
        synchronized (this) {
            for (VisionTask visionTask : taskQueue) {
                if(visionTask.getAlgorithm().equals(algorithm_.canonicalName()) && visionTask.getDescriptor().equals(Integer.toString(tag_id))) {
                    return true;
                }
            }
        }
        return false;
    }


    public List<VisionTask> visionTaskListToExecute(Algorithm algorithm_, int tag_id) {
        synchronized (this) {
            List<VisionTask> visionTaskList = new ArrayList<>();
            for (VisionTask visionTask : taskQueue) {
                if( visionTask.getAlgorithm().equals(algorithm_.canonicalName())
                        && visionTask.getDescriptor().equals(Integer.toString(tag_id))) {
                    visionTaskList.add(visionTask);
                }
            }
            return visionTaskList;
        }
    }

    public VisionTask visionTaskToExecute(Algorithm algorithm_, int tag_id) {
        synchronized (this) {
            for (VisionTask visionTask : taskQueue) {
                if(visionTask.getAlgorithm().equals(algorithm_.canonicalName()) && visionTask.getDescriptor().equals(Integer.toString(tag_id))) {
                    return visionTask;
                }
            }
        }
        return null;
    }


    //----------------------------------------------------------------------------------------------

    public boolean isThereAVisionTaskToExecute(int tag_id, String logTagTag) {
        synchronized (this) {
        for (VisionTask visionTask : taskQueue) {
//            Log.i(logTagTag,"visionTask.getDescriptor() = '"+visionTask.getDescriptor()+"', Integer.toString(tag_id) = '"+Integer.toString(tag_id)+"' ");
            if(visionTask.getDescriptor().equals(Integer.toString(tag_id))) {
//                Log.i(logTagTag,"will execute vision task "+visionTask);
                return true;
            }
        }
        }
        return false;
    }

    public VisionTask visionTaskToExecute(int tag_id) {
        synchronized (this) {
            for (VisionTask visionTask : taskQueue) {
//            Log.i(logTagTag,"visionTask.getDescriptor() = '"+visionTask.getDescriptor()+"', Integer.toString(tag_id) = '"+Integer.toString(tag_id)+"' ");
                if(visionTask.getDescriptor().equals(Integer.toString(tag_id))) {
//                Log.i(logTagTag,"will execute vision task "+visionTask);
                    return visionTask;
                }
            }
        }
        return null;
    }


    //----------------------------------------------------------------------------------------------

    public ArrayList<VisionTask> visionTasksToExecuteFilter(Algorithm algorithm_) {
        return visionTasksToExecuteFilter(algorithm_.canonicalName());
    }

    public ArrayList<VisionTask> visionTasksToExecuteFilter(String algorithm_) {
        synchronized (this) {
            ArrayList<VisionTask> tasks = new ArrayList<VisionTask>();
            for (VisionTask visionTask : taskQueue) {
                if(visionTask.getAlgorithm().contains(algorithm_)) {
                    tasks.add(visionTask);
                }
            }
            return tasks;
        }
    }


    //----------------------------------------------------------------------------------------------


    public boolean isThereAVisionTaskToExecute() {
        synchronized (this) {
            return !taskQueue.isEmpty();
        }
    }


    //----------------------------------------------------------------------------------------------

    public void addVisionTaskToQueue(WhereIsAsPub message) {
        synchronized (this) {
            VisionTask visionTask = new VisionTask()
                    .robotId(message.getRobotId())
                    .algorithm(message.getAlgorithm())
                    .descriptor(message.getDescriptor())
                    .requestId(message.getRequestId())
                    .relationToBase(message.getRelationToBase())
                    .returnUrl(message.getReturnUrl())
                    .executionIterations(message.getRate());
            System.out.println("addVisionTaskToQueue(WhereIsAsPub message) : new VisionTask="+visionTask);
            taskQueue.add(visionTask);
        }
    }

    public void addVisionTaskToQueue(WhereIsAsPubLocal message) {
        synchronized (this) {
            VisionTask visionTask = new VisionTask()
                    .algorithm(message.getAlgorithm())
                    .descriptor(message.getDescriptor())
                    .requestId(message.getRequestId())
                    .relationToBase(message.getRelationToBase())
                    .returnUrl(message.getReturnUrl())
                    .executionIterations(message.getRate());
            System.out.println("addVisionTaskToQueue(WhereIsAsPubLocal message) : new VisionTask="+visionTask);
            taskQueue.add(visionTask);
        }
    }


    //----------------------------------------------------------------------------------------------


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
            // comment out so that not removing - for quicker test setup
            taskQueue.removeAll(toRemove);
            // comment out so that not removing - for quicker test setup
        }
    }
}
