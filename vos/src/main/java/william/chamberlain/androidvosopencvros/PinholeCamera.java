package william.chamberlain.androidvosopencvros;

/**
 * Created by will on 26/09/17.
 */

public class PinholeCamera {

    public double[] project_pixel_to_world(float x_pixels, float y_pixels, int max_width, int max_height, double pose_z, float fx, float fy, float u0, float v0) {
        // y_pixels is 0 at top left of image: v0 is approx halfway up the image: assuming horizontal camera, obstacle-free pixels are between v0 and max y_pixels : project_ditances = x is +ve away from camera, goes to inf as y_pixels goes to v0
        float v = y_pixels - v0;  // v is relative to Pv ; the centre pixel : pixel halfway up the image equals v0 and projects to infinity; pixel at bottom of image is as close to the tripod as the camera can see; pixel at top of image has effective negative distance to floor
        // projected_distance/pose_z = fy/v
        double projected_distance = pose_z * (fy/v);  // in camera/robot/world coordinate frame (not image xy): distance away from camera that the ray intersects the floor assuming horizontal camera pose: projected_x is distance along the camera's/robot's x-axis


//        camera_height / projected_x  = opposite/adjacent = y_pixels / fy
//        camera_height = ( y_pixels / fy ) * projected_x
//        camera_height / ( y_pixels / fy ) = projected_x
//        double projected_x = pose_z/ ((float)y_pixels/fy)  ;  // in camera/robot/world coordinate frame (not image xy): distance away from camera that the ray intersects the floor assuming horizontal camera pose: projected_x is distance along the camera's/robot's x-axis

        // x_pixels is 0 at top left of image: u0 is approx halfway across the image: y is +ve to the left
        float u = u0 - x_pixels; // u is relative to Pu ; the centre pixel : pixel on optical axis has 0 lateral distance; pixel on left of image has maximum perceivable +ve lateral distance; pixel on right of image has maximum perceivable -ve lateral distance
        // projected_distance / projected_y = fx / u
        // projected_distance  = projected_y * (fx / u)
        // projected_distance / (fx / u)  = projected_y
        double projected_y = projected_distance / (fx / u);

        // projected_y / projected_x /* = opposite/adjacent */ = x_pixels / fx;
        // projected_y / projected_x = x_pixels / fx
        // projected_y = projected_x * (x_pixels / fx)
        //double projected_y = projected_x * ( (float)(x_pixels - (max_width/2)) / fx); // in camera/robot/world coordinate frame (not image xy): distance left or right from the camera/robot's optical axis/x-axis

        double[] pixel_to_paint = new double[4];
        pixel_to_paint[0] = max_height - projected_distance;   // +ve x distance is up/forward, pixel(0,0) is top left   // todo - scale
        pixel_to_paint[1] = (max_width/2) - projected_y;       // +ve y distance is left,       pixel(0,0) is top left   // todo - scale
        pixel_to_paint[2] = projected_distance;
        pixel_to_paint[3] = projected_y;
        return pixel_to_paint;
    }

    public static void main(String[] args) {
        PinholeCamera camera = new PinholeCamera();
        //640x480
        double[] pixel_to_paint;
        pixel_to_paint = camera.project_pixel_to_world(320,  1, 640, 480, 1.2, 519f, 518f, 320.0f, 240.0f);   output_to_terminal(pixel_to_paint);
        pixel_to_paint = camera.project_pixel_to_world(  1,  1, 640, 480, 1.2, 519f, 518f, 320.0f, 240.0f);   output_to_terminal(pixel_to_paint);
        pixel_to_paint = camera.project_pixel_to_world(640,  1, 640, 480, 1.2, 519f, 518f, 320.0f, 240.0f);   output_to_terminal(pixel_to_paint);
        pixel_to_paint = camera.project_pixel_to_world(320,100, 640, 480, 1.2, 519f, 518f, 320.0f, 240.0f);   output_to_terminal(pixel_to_paint);
        pixel_to_paint = camera.project_pixel_to_world(320,200, 640, 480, 1.2, 519f, 518f, 320.0f, 240.0f);   output_to_terminal(pixel_to_paint);
        pixel_to_paint = camera.project_pixel_to_world(320,239, 640, 480, 1.2, 519f, 518f, 320.0f, 240.0f);   output_to_terminal(pixel_to_paint);
        pixel_to_paint = camera.project_pixel_to_world(320,250, 640, 480, 1.2, 519f, 518f, 320.0f, 240.0f);   output_to_terminal(pixel_to_paint);
        pixel_to_paint = camera.project_pixel_to_world(320,300, 640, 480, 1.2, 519f, 518f, 320.0f, 240.0f);   output_to_terminal(pixel_to_paint);
        System.out.println("From 1 to 480/2 by 5s");
        for (int y_val=1; y_val<480/2; y_val=y_val+5) {
            pixel_to_paint = camera.project_pixel_to_world(320,y_val, 640, 480, 1.2, 519f,518f,320.0f, 240.0f);   output_to_terminal(pixel_to_paint);
        }
        System.out.println("From "+( (480/2)-30 )+" to 480/2 by 1s");
        for (int y_val=(480/2)-30; y_val<480/2; y_val++) {
            pixel_to_paint = camera.project_pixel_to_world(320,y_val, 640, 480, 1.2, 519f,518f,320.0f, 240.0f);   output_to_terminal(pixel_to_paint);
        }
    }

    private static void output_to_terminal(double[] pixel_to_paint) {
        System.out.println("projected_distance = "+pixel_to_paint[2]+", projected lateral distance = "+pixel_to_paint[3]+", angle = "+Math.toDegrees(Math.atan( 1.2 / pixel_to_paint[2] )));
    }
}
