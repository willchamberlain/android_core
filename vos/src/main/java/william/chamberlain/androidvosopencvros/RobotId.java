package william.chamberlain.androidvosopencvros;

import android.support.annotation.NonNull;

/**
 * Created by will on 23/06/17.
 */

public class RobotId implements java.io.Serializable, Comparable<String>, CharSequence {
    private int id_int;
    private String id;

    public RobotId(int id_){
        id_int = id_;
        id = String.valueOf(id_int);
    }

    public RobotId(String id_){
        id_int = -1;
        id = id_;
    }

    public String idString(){
        return id;
    }

    public int idInt(){
        return id_int;
    }

    @Override
    public boolean equals(Object anObject) {
        if (anObject == this) {
            return true;
        } else if(anObject instanceof RobotId) {
            return id.equals( ((RobotId)anObject).idString());
        } else {
            return false;
        }
    }

    @Override
    public int length() {
        return id.length();
    }

    @Override
    public char charAt(int index) {
        return id.charAt(index);
    }

    @Override
    public CharSequence subSequence(int start, int end) {
        return id.subSequence(start, end) ;
    }

    @Override
    public int compareTo(@NonNull String o) {
        return id.compareTo(o);
    }
}
