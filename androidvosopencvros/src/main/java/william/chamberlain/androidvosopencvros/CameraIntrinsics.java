package william.chamberlain.androidvosopencvros;

import boofcv.struct.calib.CameraPinholeRadial;

/**
 * Created by will on 3/08/17.
 */

public class CameraIntrinsics {
    static CameraPinholeRadial exampleCameraPinholeRadial() {  /*  TODO - HARDCODING  */
        return new CameraPinholeRadial(
//                    3497.344137205399, 3498.654679053389,                           // double fx, double fy
//                    0,                                                              // double skew
//                    2090.8667940953733, 1517.8452462318207,                         // double cx, double cy
//                    4160, 3120).fsetRadial(0.0673437541514291, -0.1418426025730499);
            3497.344137205399/13.0  , 3498.654679053389/13.0 ,  // 4160x3120 --> 320x240 = 13:1
                0,
                2090.8667940953733/13.0 , 1517.8452462318207/13.0 ,
                4160/13 , 3120/13 ).fsetRadial(0.0673437541514291,-0.1418426025730499);   /* TODO - HARDCODING */
    }
}
