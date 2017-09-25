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
                "0.0027867139235903207|0.03003648617801969|0.015378834791476042|0.23553022773904084|0.005411167431307971|0.05440928459893589|0.038637830297085916|0.2742452519574024|0.09227603804851736|0.2583259311575791|0.05671335966874481|0.17940019735284127|-0.09901704668963685|0.17394714568287178|0.023491151058588525|0.07646764366934307|0.00953126277763769|0.031434371939989464|-0.04842969062149675|0.2018509546184143|0.030660775807213374|0.06982288568776632|-0.0370275591417906|0.26158090489319713|0.0821175533219357|0.3277803290181747|-0.005556257998232722|0.19345942880630507|-0.08546946180892373|0.17042108577708248|-0.011759331162525881|0.17453354681759226|0.009826785107351013|0.01171141879197428|8.986009740192435E-4|0.01727842991616028|0.07143947257876047|0.07332040686016264|0.012797390313657305|0.056623040793291654|0.047789826390683535|0.36908732522973625|-0.02146913216783303|0.10485260422356987|-0.050011913272657305|0.1389876797608195|-0.0414591330271695|0.13114412338262893|0.025187043705651505|0.0478549049809381|0.030024984754366518|0.08234355877887607|0.08257157583368585|0.09969989962422528|0.02375423798900787|0.13707618299825425|-0.014292772605949133|0.30377967664589345|0.013916719939028287|0.10758902547150936|-0.03885767713450634|0.08105658017488425|0.0058101740893761545|0.06353977107238168",
                "7.261367176873209E-4|0.0028002471977859457|6.745317500399117E-4|0.002942658142202091|0.05822066000832679|0.05986157265183892|0.010702760025801282|0.012311882435055958|0.09898633048820524|0.308752916608069|0.01443422069187943|0.03907895614995743|-0.008893508459966996|0.2622196360619047|-0.004112176598869702|0.03513540838546368|-3.0592336384504024E-4|0.004378248529036454|-0.0015064966144884098|0.0042166311423248734|0.10627024934530706|0.10946913956613573|0.016336831938315866|0.01933332874834842|0.07101375211367536|0.37858284160884653|0.013236929903893377|0.031514201111049134|0.019321098048875354|0.29194735005224415|-0.0013495462340034558|0.026023036450237336|-9.005831069323092E-4|0.006110391252555429|-0.0014263961701023935|0.00413874071401758|0.13054122933417686|0.1580259504136574|0.012043529884001935|0.01872547756300193|0.008282472636410868|0.40809464566854686|0.0011698713500161522|0.022412495846307837|0.01638607276466247|0.296206956620337|-0.002283215117449256|0.023579412453084134|-0.002209666678321142|0.005885070181202123|2.5053620211516774E-4|0.003131271889113173|0.12919161237689142|0.17549470090588898|0.005870536436285037|0.019008778433921934|0.013490178446666435|0.38279179780445655|0.002922483769702581|0.027502710352348206|0.031822387994146925|0.24660150975541684|0.005524393885463752|0.022333742868528857",
                "0.003731963966617743|0.00422416556131915|-0.0033125538995692257|0.0037337959359848177|0.13563842400044165|0.17613883469981237|-0.008680476819861425|0.017214397780342543|0.02103267379319336|0.3533469246300399|-0.0028040519819491376|0.02683121212565505|0.02644681265618982|0.220759687413325|-0.004207605377995653|0.024241835319792602|0.004451753392076792|0.004858044463309035|-0.0034403568092147064|0.004000426560326805|0.158598435588286|0.18080792458723358|-0.013591154136730198|0.016175575341167283|0.05004064188765949|0.38961546074911785|-0.0043076652863514|0.018536450696761162|0.032398056309681955|0.26317802046518524|0.004589082668819666|0.01830476726631106|0.0059508232547791666|0.0060651837110548565|-0.002797681976198934|0.003355618863252081|0.13781646280351753|0.14082900065444034|-0.0201595023017308|0.020959857657773886|0.07970733728052252|0.3983063523635948|-0.012121940937521596|0.025137922795283794|0.020694246135331378|0.2904801686002399|-6.771568883916708E-4|0.018437823002826238|0.004789397758192172|0.005010486615520201|-0.0021432010190511634|0.00317098557689811|0.08026482762868092|0.08026482762868092|-0.01597241439093791|0.016418367747372396|0.08091182503434127|0.3228183212468336|-0.01813334396074223|0.03869046432710124|-0.011650801249856861|0.2698814150116975|-0.005606407770007395|0.02524754544280499",
                "0.01681086766598422|0.2589888146237019|-0.0012326606736299478|0.04105492094760681|0.008845392104637605|0.2812158353033896|0.0023472475657490416|0.036927796431998706|0.13429603555164638|0.1736916463323205|-0.005876458345861575|0.020800386672776828|-0.007505355478620101|0.053002592494636604|-3.4621778089669E-4|0.012041290003648968|0.03240593646179572|0.2914481603550686|0.005830172731024612|0.04574819356955454|-0.004606004046134467|0.3363696283706842|0.002488447414078854|0.04405318484882269|0.146579406552533|0.22631267760268445|-0.01976514927579126|0.03163757513848129|0.006032678098779911|0.07040388833128482|-0.0018125535453572701|0.01655262291638877|0.06213977394548995|0.2953401863617023|0.0035508617358099544|0.039602610556241806|-0.038138505087330146|0.34476783595040505|0.014512759587599813|0.04392575922004392|0.11032175906298039|0.2501312093439723|-0.01986873914541454|0.04193096478936297|0.03268412780189469|0.08618269971344773|-0.007531200522500876|0.018591182537884082|0.07306421340629338|0.22567398180306625|-0.01278668173944826|0.031500611589893944|-0.043578404761346314|0.29238169038453243|0.009750259778782567|0.038651944336160225|0.05003478334437132|0.2295613576690996|-0.0077378497352828535|0.04232714292480167|0.059784920216881234|0.08817855161566301|-0.01124726920510303|0.01743120264538677",
                "5.957958293449052E-4|0.003054417904877124|-0.0011821749262238205|0.0032642995290239896|0.11105019938517915|0.1690995397438152|-0.0019430179226941062|0.024532751379483284|0.021512849049643654|0.3540725480710722|0.015476038438025681|0.04186046291089394|0.044046165254733295|0.23681410770796338|-2.3001795536695127E-4|0.024453724823587663|0.0023618277589692693|0.0049764860853468715|-0.0020695564087435145|0.00446546853356425|0.13501048195784998|0.17118114523860517|-0.014021138920925406|0.020613411454899387|0.02008842042935601|0.42144537734638804|-5.916538122825998E-4|0.02990629188218011|0.024446613866234916|0.27975869979501045|0.0010824275249870347|0.02641554616918257|6.569545402523497E-5|0.0036787310853214996|8.254913518556608E-4|0.003555536214170759|0.11048109851258425|0.11718966948525948|-0.01524066030269327|0.01900951080187671|0.022625479087430265|0.3882539453962733|-0.0024278376708256914|0.03683003863615075|-0.006378584500502465|0.3013508509996905|0.009290548508586542|0.036515792616533045|-5.005570523532538E-4|0.0034914802951261136|2.6823516305379575E-4|0.003019153489317712|0.07108523240261237|0.07185540629406142|-0.011845050390561558|0.01705846996064902|0.07749175277983823|0.32154824406832105|-0.012365403452976589|0.0550806359734361|-0.017717078843561388|0.26147566924621946|0.007636072292425879|0.043333932558134496"
        };
    }
}