package william.chamberlain.androidvosopencvros;

/**
 * Created by will on 8/08/17.
 */

public interface RosThingy {
    void    registerAsVisionSource();
    int     getCamNum();
    void reportDetectedFeature(int tagId, double x, double y, double z, double qx, double qy, double qz, double qw);
    void updateLocationFromDetectedFeature(int tagId, double x, double y, double z, double qx, double qy, double qz, double qw);


    }
