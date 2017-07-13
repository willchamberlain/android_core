package william.chamberlain.androidvosopencvros;

import org.ros.node.topic.Subscriber;

import vos_aa1.WhereIsAsPub;

/**
 * Created by will on 13/07/17.
 */

public class WhereIsSubscriber {
    Subscriber<WhereIsAsPub> subscriber;
    MainActivity activity;

    public WhereIsSubscriber(MainActivity activity_) {
        activity = activity_;
    }


    public void setSubscriber(Subscriber<WhereIsAsPub> subscriber) {
        this.subscriber = subscriber;
    }

    public void called(WhereIsAsPub message){
        activity.dealWithRequestForInformation(message);
    }
}
