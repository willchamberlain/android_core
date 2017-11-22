package william.chamberlain.androidvosopencvros;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Subscriber;

import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import std_msgs.String;
import william.chamberlain.androidvosopencvros.device.DimmableScreen;

/**
 * Listens for management commands on the /management topic.
 *
 * Created by will on 1/03/17.
 */

public class VisionSourceManagementListener extends AbstractNodeMain {
    private Log log;
    private Subscriber<String> subscriber;
    private DimmableScreen dimmableScreen;
    private VariableResolution variableResolution;
    private VisionSource visionSource;
    private java.lang.String nodeNamespace;


    public void setVariableResolution(VariableResolution variableResolution) {
        this.variableResolution = variableResolution;
    }

    public void setDimmableScreen(DimmableScreen dimmableScreen) {
        this.dimmableScreen = dimmableScreen;
    }

    public void setNodeNamespace(java.lang.String nodeNamespace) {
        this.nodeNamespace = nodeNamespace;
    }

    /*
            The NodeMain.getDefaultNodeName method returns the default name of the node.
            This name will be used unless a node name is specified in the NodeConfiguration (more on that later). GraphNames are used throughout rosjava when refering to nodes, topics, and parameters. Most methods which accept a GraphName will also accept a string for convenience.
            */
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("VisionSourceManagementListener");
    }

    public static final Pattern commandPattern = Pattern.compile("([A-z0-9_]+) +> +([A-z0-9_ ]+)");   // "all > screen off", "all > screen on"

    public java.lang.String[] parse(java.lang.String command) {
        log.debug("Command = '"+command+"'");
        Matcher matcher = commandPattern.matcher(command);
        ArrayList<java.lang.String> commandParts = new ArrayList<java.lang.String>();

        matcher = commandPattern.matcher(command);
        for (int i_ = 0; matcher.find(); i_++) {
            log.debug("  match "+i_+" matches this sequence '"+matcher.group(0)+"'");
            for (int j_ = 1; matcher.groupCount() >= j_; j_++) {
                log.debug("    "+matcher.group(j_));
                commandParts.add(matcher.group(j_));
            }
        }
        return commandParts.toArray(new java.lang.String[1]);
    }


    /*
    The NodeListener.onStart method is the entry point for your program (or node).
    The ConnectedNode parameter is the factory we use to build things like Publishers and Subscribers.
     */
    @Override
    public void onStart(ConnectedNode connectedNode) {
        log = connectedNode.getLog();
        subscriber = connectedNode.newSubscriber("management", std_msgs.String._TYPE);
        subscriber.addMessageListener(new MessageListener<String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                java.lang.String commandsFromTopic = message.getData();
                log.info("management message received: \"" + commandsFromTopic + "\"");
                java.lang.String[] commandParts = parse(commandsFromTopic);
//  TODO - check that the first part of the command matches this camera name e.g. c1
                if(null == commandParts || 2 > commandParts.length) {
                    log.warn("management message not matched to pattern: \"" + commandsFromTopic + "\"");
                    return;
                }
                if (commandParts[0].equals("all") || commandParts[0].equals(nodeNamespace)) {
                    ManagementCommand command = ManagementCommand.findByKey(commandParts[1]);
                    if (null == command) {
                        log.warn("management message command \"" + commandParts[1] + "\" not recognised.");
                        return;
                    }
                    switch (command) {
                        case SCREEN_ON:
                            dimmableScreen.screenOn();
                            break;
                        case SCREEN_ON_PREFERRED:
                            dimmableScreen.screenOn(-1.0F);
                            break;
                        case SCREEN_ON_BRIGHT:
                            dimmableScreen.screenOn(1.0F);
                            break;
                        case SCREEN_ON_MID:
                            dimmableScreen.screenOn(0.6F);
                            break;
                        case SCREEN_ON_LOW:
                            dimmableScreen.screenOn(0.3F);
                            break;
                        case SCREEN_OFF:
                            dimmableScreen.screenOff();
                            break;
                        case PROCESSING_ON:
                            dimmableScreen.screenOn();
                            visionSource.start();
                            break;
                        case PROCESSING_OFF:
                            dimmableScreen.screenOff();
                            visionSource.stop();
                            break;
                        case OBSTACLE_DETECTION_ON:
                            dimmableScreen.screenOn();
                            visionSource.startObstacleDetection();
                            break;
                        case OBSTACLE_DETECTION_ON_HSV:
                            dimmableScreen.screenOn();
                            visionSource.startObstacleDetectionHSV();
                            break;
                        case OBSTACLE_DETECTION_ON_TEXTURE:
                            dimmableScreen.screenOn();
                            visionSource.startObstacleDetectionTexture();
                            break;
                        case OBSTACLE_DETECTION_PROJECT:
                            dimmableScreen.screenOn();
                            visionSource.startObstacleDetectionAndProject();
                            break;
                        case OBSTACLE_DETECTION_PROJECT_HIST:
                            dimmableScreen.screenOn();
                            visionSource.startObstacleDetectionAndProjectHistory();
                            break;
                        case OBSTACLE_DETECTION_OFF:
                            dimmableScreen.screenOff();
                            visionSource.stopObstacleDetection();
                            break;
                        case RESOLUTION_VERY_VERY_HIGH:
                            variableResolution.resolutionMinMax(650,490,960,720); // --> 960x720
                            break;
                        case RESOLUTION_VERY_HIGH:
                            variableResolution.resolutionMinMax(490,370,650,490); // --> 640x480
                            break;
                        case RESOLUTION_HIGH:
                            variableResolution.resolutionMinMax(320,240,490,370); // --> 480X360
                            break;
                        case RESOLUTION_LOW:
                            variableResolution.resolutionMinMax(100,100,400,300); // --> 320x240
                            break;
                        case RESOLUTION_LIMITS:
                            variableResolution.resolutionMinMax(100,100,700,550);
                            break;
                        case RELOCALISE:
                            visionSource.relocalise();
                            break;
                        case DISPLAY_RGB:
                            dimmableScreen.displayRgb();
                            break;
                        case DISPLAY_GREY:
                            dimmableScreen.displayGrey();
                            break;
                        case ALLOCATE_TO_ROBOT:
                            visionSource.allocateTo(VisionSource.ROBOT_ALLOCATION_KEY);
                            break;
                        case ALLOCATE_TO_TARGET:
                            visionSource.allocateTo(VisionSource.TARGET_ALLOCATION_KEY);
                            break;
                        case RESET_EXTERNAL_CALIBRATION:
                            visionSource.resetExtrinsicCalibration();
                            break;
                        default:
                            log.warn("management message command not recognised: \"" + commandsFromTopic + "\"");
                            break;
                    }
                }
            }
        });
    }

    /*
    The NodeListener.onShutdown method is the first exit point for your program.
    It will be executed as soon as shutdown is started (i.e.
        before all publishers, subscribers, etc. have been shutdown).
    The shutdown of all created publishers, subscribers, etc.
        will be delayed until this method returns or the shutdown timeout expires.
     */
    @Override
    public void onShutdown(Node node) {
        log.info("VisionSourceManagementListener: onShutdownComplete");
    }

    /*
    The NodeListener.onShutdownComplete method is the final exit point for your program.
    It will be executed after all publishers, subscribers, etc. have been shutdown.
    This is the preferred place to handle clean up since it will not delay shutdown.
     */
    @Override
    public void onShutdownComplete(Node node) {
        log.info("VisionSourceManagementListener: onShutdownComplete");
    }

    /*
    The NodeListener.onError method is called when an error occurs in the Node itself.
    These errors are typically fatal.
    The NodeListener.onShutdown and NodeListener.onShutdownComplete methods will be called following the call to NodeListener.onError.
     */
    @Override
    public void onError(Node node, Throwable throwable) {
        log.error("VisionSourceManagementListener: onError.  Node="+node.getName());
        throwable.printStackTrace();
    }

    public void setVisionSource(VisionSource visionSource) {
        this.visionSource = visionSource;
    }
}
