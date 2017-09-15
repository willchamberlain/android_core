package william.chamberlain.androidvosopencvros;

import java.util.HashMap;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;

public class Hardcoding {


    public static final int MARKER_OFFSET_INT = 90000;
    public static final double BOOFCV_MARKER_SIZE_M = 0.257; //0.189;  // 0.20  // 0.14             ////  TODO - list of tags and sizes, and tag-groups and sizes


    public static void hardcodeTargetMarkers(VosTaskSet vosTaskSet) {
        // robot target marker
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "210", 1));
        // robot model markers
//        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "170", 1,  0.10,  0.00, 0.42,  0,    0,    0,          1 ));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "170", 1,  -0.05, 0.00, 0.79,  0.0d, 0.0d,  0.382696d, 0.923874d ));  // upper left  - 79 cm up, 45 deg left
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "250", 1,  -0.05, 0.00, 0.79,  0.0d, 0.0d, -0.382696d, 0.923874d ));  // upper right - 79 cm up, 45 deg right
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "290", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "330", 1));
        // fixed-position markers for PnP
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "650", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "690", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "730", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "770", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "810", 1));

        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "850", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "930", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1090", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1010", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "970", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1050", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "890", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1250", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1210", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1170", 1));

        /*** Quad for boofcv P3P/PnP  ***/
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "970", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1210", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1250", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1170", 1));


        /** check Nexus6 accuracy ***/
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1610", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1690", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1730", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "1650", 1));

        /** big markers for distance ***/
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv",  "57", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "157", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "257", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "357", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "457", 1));
        vosTaskSet.addVisionTaskToQueue(new WhereIsAsPubLocal("boofcv", "557", 1));
    }


    /* Dev: part of robot visual model */
    public static boolean isPartOfRobotVisualModel(final int tag_id) {
        return tag_id == 170 || tag_id == 250 || tag_id == 290 || tag_id == 330
                || tag_id == 1650
                || tag_id == 57 || tag_id == 157 || tag_id == 257 || tag_id == 357 || tag_id == 457 || tag_id == 557
                ;
    }

    /* Dev: part of knowledge about landmarks */
    public static boolean isALandmark(final int tag_id) {
        return tag_id == 650 || tag_id == 690 || tag_id == 730 || tag_id == 770 || tag_id == 810 || tag_id == 850 || tag_id == 930 || tag_id == 1090 || tag_id == 1010 || tag_id == 970 || tag_id == 1050 || tag_id == 890 || tag_id == 1250 || tag_id == 1210 || tag_id == 1170
                || tag_id == 1610 || tag_id == 1690 || tag_id == 1730 || tag_id == 170
                ;
    }

    public static boolean isAQuad(final int tag_id) {
        return tag_id == 970 || tag_id == 1210 || tag_id == 1250 || tag_id == 1170;
    }


    private static HashMap<Integer, double[][]> fixedCameraPoses = new HashMap<Integer, double[][]>(1);

    private static void initialiseFixedCameraPoses() {
        fixedCameraPoses.put(607, new double[][]{                   // double[] poseXyz, double[] orientationQuaternionXyzw_
                {19.6833705902d, -3.55735754967d,  1.2d     },
                {0d,  0d,  0.906324d,   0.422583d}});
    }

    public static void fixedCameraPose(int camNum_, PosedEntity posedEntity) {
        double[][] pose = fixedCameraPoses.get(camNum_);
        if(null != pose) {
            posedEntity.setPose(pose[0],pose[1]);                   // double[] poseXyz, double[] orientationQuaternionXyzw_
        }
    }


    // see   /mnt/nixbig/downloads/boofcv_and_related/lessthanoptimal/_boofcv_checkout_recursive_20170905/boofcv/examples/src/main/java/boofcv/examples/features/ExampleAssociatePoints.java
    public static String[] testSurfFeatureDescriptors(){
        return new String[] {  // describe image B using pre-recorded interest points
                "0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.007102340181796514|0.002968400569225131|0.003536344386194378|6.413197455182649E-4|0.0|0.0|0.0|0.0026152155514907725|0.058160220167282554|0.09161601006725992|0.02200954350053984|0.019486073735703886|0.0|0.005917344653233395|0.004953992696410381|0.007845646654472316|0.0021133296257928305|0.017241343076897394|0.020851728478613697|0.0062815846633952075|0.0|0.001972448217744465|0.0016513308988034602|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.03329642375657542|0.08886836717938294|0.041762677777164386|0.06740976551032625|0.04374500619902012|0.021314143001737028|9.924331868196034E-4|0.00533917794642879|0.18829472031564345|0.24143244123751764|0.23501389289736488|0.06144869661666312|0.009318527385831613|0.04349916668553758|0.10187254764514414|0.09030795067456303|0.08839522208430291|0.1922298515231803|0.24106812200818603|0.014822553949115375|0.005162934437642077|0.02349279473606289|0.03448705020944571|0.056774874449486236",
                "0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.012433477792197311|6.321433741143553E-4|0.025846346621130417|0.001825658828586132|0.05976758842638602|0.08893067460978744|0.03700066153713065|0.07956154055600859|0.20879280791310115|0.035061500828183496|0.00861544887371014|0.0161118454562524|0.17799626767605903|0.13092170077125193|0.045649060612291875|0.03881927048405425|0.012527934675728707|0.007987235011596622",
                "0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.043309854728335796|0.05947614433584635|0.029489187044784426|0.014316539646009662|0.02882771811747888|0.0071053011114571496|0.02744104426873782|0.018143124412949352|0.019818192169926828|0.03948294020900387|0.06344256387394694|0.23146321745512607|0.1602786646657306|0.015822683868241337|0.01845692143493136|0.005180491146119585|0.03341566660382575|0.010629477507206768|0.0|0.04968428130723066|0.029267958421589337|0.039312572856406824|0.0236655390376481|0.0|0.009286266604639564|0.0026304911589266535|0.0|0.0|0.0|0.0|0.0|0.0",
                "0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.04641851361456718|0.026238199885605808|0.02765251769228965|0.004751830609593564|0.0043140141719051|0.0|0.0|0.011740171892622397|0.03135532563905452|0.09753067663314707|0.06981595613468942|0.005037344409279282|0.012942042515715301|9.540403043377993E-4|0.018793597283870297|0.007829934202359728|0.03798613497273225|0.1854487855975142|0.054491381850250625|0.0|0.0|6.552950176991383E-4|0.029465524200662285|0.017418441837965117"
        };
    }
}