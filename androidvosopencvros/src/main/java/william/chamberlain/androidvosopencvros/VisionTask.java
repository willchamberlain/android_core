package william.chamberlain.androidvosopencvros;

import geometry_msgs.Pose;

/**
 * Created by will on 17/07/17.
 */

public class VisionTask {
    private String algorithm;
    private String descriptor;
    private String requestId;
    private geometry_msgs.Pose relationToBase;
    private String returnUrl;
    private int numberOfTimesToExecute=0;
    private int executions=0;

    public VisionTask(String algorithm, String descriptor, String requestId, Pose relationToBase, String returnUrl) {
        this.algorithm = algorithm;
        this.descriptor = descriptor;
        this.requestId = requestId;
        this.relationToBase = relationToBase;
        this.returnUrl = returnUrl;
        numberOfTimesToExecute=1;
    }

    public VisionTask(){
        numberOfTimesToExecute=1;
    }

    public void executed() {
        executions++;
    }

    public boolean canBeExecuted() {
        return numberOfTimesToExecute > executions;
    }

    public String getAlgorithm() {
        return algorithm;
    }

    public VisionTask algorithm(String algorithm) {
        this.algorithm = algorithm;
        return this;
    }

    public String getDescriptor() {
        return descriptor;
    }

    public VisionTask descriptor(String descriptor) {
        this.descriptor = descriptor;
        return this;
    }

    public String getRequestId() {
        return requestId;
    }

    public VisionTask requestId(String requestId) {
        this.requestId = requestId;
        return this;
    }

    public Pose getRelationToBase() {
        return relationToBase;
    }

    public VisionTask relationToBase(Pose relationToBase) {
        this.relationToBase = relationToBase;
        return this;
    }

    public String getReturnUrl() {
        return returnUrl;
    }

    public VisionTask returnUrl(String returnUrl) {
        this.returnUrl = returnUrl;
        return this;
    }

    public VisionTask executionIterations(int numberOfTimesToExecute_) {
        numberOfTimesToExecute = numberOfTimesToExecute_;
        return this;
    }

    @Override
    public String toString() {
        return "VisionTask{" +
                "algorithm='" + algorithm + '\'' +
                ", descriptor='" + descriptor + '\'' +
                ", requestId='" + requestId + '\'' +
                ", relationToBase=" + relationToBase +
                ", returnUrl='" + returnUrl + '\'' +
                ", numberOfTimesToExecute=" + numberOfTimesToExecute +
                ", executions=" + executions +
                '}';
    }
}
