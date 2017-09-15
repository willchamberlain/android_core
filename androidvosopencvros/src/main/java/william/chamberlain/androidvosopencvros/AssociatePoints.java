package william.chamberlain.androidvosopencvros;
/*
 * Copyright (c) 2011-2017, Peter Abeles. All Rights Reserved.
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


        import boofcv.abst.feature.associate.AssociateDescription;
        import boofcv.abst.feature.associate.ScoreAssociation;
        import boofcv.abst.feature.detdesc.DetectDescribePoint;
        import boofcv.abst.feature.detect.interest.ConfigFastHessian;
        import boofcv.alg.descriptor.UtilFeature;
        import boofcv.factory.feature.associate.FactoryAssociation;
        import boofcv.factory.feature.detdesc.FactoryDetectDescribe;
        import boofcv.gui.feature.AssociationPanel;
        import boofcv.gui.image.ShowImages;
        import boofcv.io.image.ConvertBufferedImage;
        import boofcv.io.image.UtilImageIO;
        import boofcv.struct.feature.AssociatedIndex;
        import boofcv.struct.feature.TupleDesc;
        import boofcv.struct.image.GrayF32;
        import boofcv.struct.image.ImageGray;
        import georegression.struct.point.Point2D_F64;
        import org.ddogleg.struct.FastQueue;

        import boofcv.struct.image.Planar;
        //import boofcv.struct.feature.SurfFeature;

//        import java.awt.image.BufferedImage;
        import java.util.ArrayList;
        import java.util.List;

        import java.util.regex.Pattern;


/**
 * After interest points have been detected in two images the next step is to associate the two
 * sets of images so that the relationship can be found.  This is done by computing descriptors for
 * each detected feature and associating them together.  In the code below abstracted interfaces are
 * used to allow different algorithms to be easily used.  The cost of this abstraction is that detector/descriptor
 * specific information is thrown away, potentially slowing down or degrading performance.
 *
 * @author Peter Abeles
 */
public class AssociatePoints<T extends ImageGray<T>, TD extends TupleDesc> {

    // algorithm used to detect and describe interest points
    DetectDescribePoint<T, TD> detDesc;
    // Associated descriptions together by minimizing an error metric
    AssociateDescription<TD> associate;

    // location of interest points
    public List<Point2D_F64> pointsA;
    public List<Point2D_F64> pointsB;

    Class<T> imageType;
    Class<T> imageRgbType;

    public AssociatePoints(DetectDescribePoint<T, TD> detDesc,
                           AssociateDescription<TD> associate,
                           Class<T> imageType) {
        this.detDesc = detDesc;
        this.associate = associate;
        this.imageType = imageType;
    }

    //(detDesc,associate,imageTypeGrayF32, imageRgbType)
    public AssociatePoints(DetectDescribePoint<T, TD> detDesc,
                           AssociateDescription<TD> associate,
                           Class<T> imageType, Class<T> imageRgbType) {
        this.detDesc = detDesc;
        this.associate = associate;
        this.imageType = imageType;
        this.imageRgbType = imageRgbType;
    }


//    public void associateString( T inputA, String[] featureString, BufferedImage imageA)
//    {
//        associateSurfAndString(inputA, featureString);
//        //---------------------------------------------------------------------------------
//        AssociationPanel panel = new AssociationPanel(20); // display the results
//        panel.setAssociation(pointsA,pointsB,associate.getMatches());
//        panel.setImages(imageA,imageA);
//        //------
//        ShowImages.showWindow(panel,"Associated Features",true);
//    }

    class ReturnValue {
        List<Point2D_F64> pointsA;
        List<Point2D_F64> pointsB;
        FastQueue<AssociatedIndex> associate_getMatches;

        public ReturnValue(List<Point2D_F64> pointsA, List<Point2D_F64> pointsB, FastQueue<AssociatedIndex> associate_getMatches) {
            this.pointsA = pointsA;
            this.pointsB = pointsB;
            this.associate_getMatches = associate_getMatches;
        }
    }

    // from AssociationPanel
    public void drawPoints( List<Point2D_F64> leftPts , List<Point2D_F64> rightPts,
                         FastQueue<AssociatedIndex> matches ) {
        int assocLeft[],assocRight[];    // which features are associated with each other
        List<Point2D_F64> allLeft = new ArrayList<>();
        List<Point2D_F64> allRight = new ArrayList<>();
        assocLeft =new int[matches.size()];
        assocRight =new int[matches.size()];
        for( int i = 0; i<matches.size();i++)
        {
            AssociatedIndex a = matches.get(i);
            allLeft.add(leftPts.get(a.src));
            allRight.add(rightPts.get(a.dst));
            assocLeft[i] = i;
            assocRight[i] = i;
        }
        for( int i = 0; i < assocLeft.length; i++ ) {
            if( assocLeft[i] == -1 ) {
                continue;
            }
            Point2D_F64 l = leftPts.get(i);
            Point2D_F64 r = rightPts.get(assocLeft[i]);
            //            Color color = colors[i];
            //            drawAssociation(g2, scaleLeft,scaleRight,rightX, l, r, color);
        }
    }

    public ReturnValue associateSurfAndString(T inputA, String[] featureString) {
        pointsA = new ArrayList<>();  // stores the location of detected interest points
        pointsB = new ArrayList<>();
        //------
        FastQueue<TD> descA = UtilFeature.createQueue(detDesc,100);  // stores the description of detected interest points
        FastQueue<TD> descB = UtilFeature.createQueue(detDesc,100);
        //------
        describeImage(inputA,pointsA,descA); // describe image A using interest points
        //---
        describeImageFromFixedList(pointsB,descB,featureString); // describe image B using pre-recorded interest points
        //------
        associate.setSource(descA);  // Associate features between the two images
        associate.setDestination(descB);
        associate.associate();
        ReturnValue returnValue = new ReturnValue(pointsA, pointsB, associate.getMatches());
        return returnValue;
    }

//    public void associateString( BufferedImage imageA, String[] featureString)
//    {
//        T inputA = ConvertBufferedImage.convertFromSingle(imageA, null, imageType);   //  note:  static <T extends ImageGray<T>> T 	convertFromSingle(BufferedImage src, T dst, Class<T> type)
//        associateString( inputA, featureString, imageA);
//    }

    /**
     * Detects features inside the two images and computes descriptions at those points.
     */
    private void describeImage(T input, List<Point2D_F64> points, FastQueue<TD> descs )
    {
        detDesc.detect(input);

        for( int i = 0; i < detDesc.getNumberOfFeatures(); i++ ) {
            points.add( detDesc.getLocation(i).copy() );
            descs.grow().setTo(detDesc.getDescription(i));

            System.out.println("Point:"+detDesc.getLocation(i));

            String concatenated_descriptor="";
            for( int subregion_ = 0; subregion_ < 64; subregion_++) {
                if(subregion_>0) {concatenated_descriptor=concatenated_descriptor+"|";}
                concatenated_descriptor = concatenated_descriptor + ((boofcv.struct.feature.BrightFeature)detDesc.getDescription(i)).value[subregion_];
            }
            System.out.println("Feature "+i+": descriptor="+concatenated_descriptor);
        }
    }


    private void describeImageFromFixedList(List<Point2D_F64> points, FastQueue<TD> descs, String[] fixedListOfSurfDescriptors)
    {
        Pattern p = Pattern.compile("\\|");
        for(int desc_num_=0 ; desc_num_ < fixedListOfSurfDescriptors.length ; desc_num_++) {
            System.out.println("SURF features: describeImageFromFixedList: fixedListOfSurfDescriptors.length["+desc_num_+"]");
            String[] descriptors = p.split(fixedListOfSurfDescriptors[desc_num_]);
            points.add(new Point2D_F64(10.0+10.0*(float)desc_num_ , 10.0+10.0*(float)desc_num_ ));
            boofcv.struct.feature.BrightFeature desc_BrightFeature = new boofcv.struct.feature.BrightFeature(64);
            double[] values = new double[64];
            for(int i_=0 ; i_ < descriptors.length ; i_++) {
                values[i_]=Double.parseDouble(descriptors[i_]);
//                System.out.println("SURF features: describeImageFromFixedList: descriptors["+i_+"]");
            }
            System.out.println("SURF features: describeImageFromFixedList: desc_BrightFeature.value.length="+desc_BrightFeature.value.length);
            System.out.println("SURF features: describeImageFromFixedList: values.length="+values.length);
            desc_BrightFeature.white=true;
            desc_BrightFeature.setValue(values);
            descs.grow().setTo(desc_BrightFeature);
        }

    }




    private void describeImageSurf(T input, List<Point2D_F64> points, FastQueue<TD> descs )
    {
        detDesc.detect(input);

        for( int i = 0; i < detDesc.getNumberOfFeatures(); i++ ) {
            points.add( detDesc.getLocation(i).copy() );
            descs.grow().setTo(detDesc.getDescription(i));
        }
    }



//    public static void main( String args[] ) {
//
//        Class imageTypeGrayF32 = GrayF32.class;
//        //		Class imageTypeGrayU8 = GrayU8.class;
//        Class imageRgbType = Planar.class;
//
//        // select which algorithms to use
//        DetectDescribePoint detDesc = FactoryDetectDescribe.surfStable(new ConfigFastHessian(1, 2, 300, 1, 9, 4, 4), null,null, imageTypeGrayF32);
//        //		        sift(new boofcv.abst.feature.detdesc.ConfigCompleteSift(0,4,600));
//        //		        surfColorStable(new ConfigFastHessian(1, 2, 300, 1, 9, 4, 4), null,null, imageRgbType);
//        //				surfStable(new ConfigFastHessian(1, 2, 300, 1, 9, 4, 4), null,null, imageTypeGrayF32);
//        //				sift(new boofcv.abst.feature.detdesc.ConfigCompleteSift(0,4,600));
//        //				sift(new boofcv.abst.feature.detdesc.ConfigCompleteSift(0,4,200));
//
//        // is a Euclidean distance scorer - Euclidean distance between descriptor vectors
//        ScoreAssociation scorer = FactoryAssociation.defaultScore(detDesc.getDescriptionType());
//        System.out.println("----- ScoreAssociation scorer = "); System.out.println(scorer.getClass());  System.out.println("----------------------");
//        AssociateDescription associate = FactoryAssociation.greedy(scorer, Double.MAX_VALUE, true);
//
//        // load and match images
//        AssociatePoints app = new AssociatePoints(detDesc,associate,imageTypeGrayF32, imageRgbType);
//        BufferedImage imageA = null;
//        BufferedImage imageB = null;
//        imageA = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/poster_20170912_095137_cropped.jpg");
//        imageB = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/poster_20170912_095153_cropped.jpg");
//
//        GrayF32 imageGrayA = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/poster_20170912_095137_cropped.jpg", GrayF32.class);
//        GrayF32 imageGrayB = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/poster_20170912_095153_cropped.jpg", GrayF32.class);
//
//        //		BufferedImage imageA = UtilImageIO.loadImage(UtilIO.pathExample("stitch/kayak_01.jpg"));
//        //		BufferedImage imageB = UtilImageIO.loadImage(UtilIO.pathExample("stitch/kayak_03.jpg"));
//        //        BufferedImage imageA = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/poster_20170912_095137_cropped.jpg");
//        //        BufferedImage imageA = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/documents/ACRV_templates/ACRV_logo/ACRV_logo_2017_09_11_RV_LOGO_COLOUR.jpg");
//        //		inputs.add(new PathLabel("ACRV cropped 3", "/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/acrv_20170911_151649_cropped.jpg"));
//        //		BufferedImage imageA = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/acrv_20170911_151638_cropped.jpg");
//        //		BufferedImage imageB = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/acrv_20170911_151659_cropped.jpg");
//        //		BufferedImage imageB = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/acrv_20170911_151638.jpg");
//        //		BufferedImage imageB = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/acrv_20170911_151659.jpg");
//        //		BufferedImage imageB = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/acrv_20170911_151654.jpg");
//        //		BufferedImage imageB = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/acrv_20170911_151649_cropped.jpg");
//        //		BufferedImage imageB = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/poster_20170912_095137.jpg");
//        //		BufferedImage imageB = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/poster_20170912_095153.jpg");
//        //		BufferedImage imageB = UtilImageIO.loadImage("/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_09_11_ACRV_logos/poster_20170912_095153_cropped.jpg");
//
//        //		associateSurfBasic(imageGrayA, imageGrayB);     -- broken - needs imports of SurfFeature from V4J4 or some such
//        //		app.associateRgb(imageA,imageB);                --  broken - no associations come out
//
////		app.associate(imageA,imageB);   // -- works!!
//
//        String[] imageBfeatureString = new String[] {  // describe image B using pre-recorded interest points
//                "0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.007102340181796514|0.002968400569225131|0.003536344386194378|6.413197455182649E-4|0.0|0.0|0.0|0.0026152155514907725|0.058160220167282554|0.09161601006725992|0.02200954350053984|0.019486073735703886|0.0|0.005917344653233395|0.004953992696410381|0.007845646654472316|0.0021133296257928305|0.017241343076897394|0.020851728478613697|0.0062815846633952075|0.0|0.001972448217744465|0.0016513308988034602|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.03329642375657542|0.08886836717938294|0.041762677777164386|0.06740976551032625|0.04374500619902012|0.021314143001737028|9.924331868196034E-4|0.00533917794642879|0.18829472031564345|0.24143244123751764|0.23501389289736488|0.06144869661666312|0.009318527385831613|0.04349916668553758|0.10187254764514414|0.09030795067456303|0.08839522208430291|0.1922298515231803|0.24106812200818603|0.014822553949115375|0.005162934437642077|0.02349279473606289|0.03448705020944571|0.056774874449486236",
//                "0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.012433477792197311|6.321433741143553E-4|0.025846346621130417|0.001825658828586132|0.05976758842638602|0.08893067460978744|0.03700066153713065|0.07956154055600859|0.20879280791310115|0.035061500828183496|0.00861544887371014|0.0161118454562524|0.17799626767605903|0.13092170077125193|0.045649060612291875|0.03881927048405425|0.012527934675728707|0.007987235011596622",
//                "0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.043309854728335796|0.05947614433584635|0.029489187044784426|0.014316539646009662|0.02882771811747888|0.0071053011114571496|0.02744104426873782|0.018143124412949352|0.019818192169926828|0.03948294020900387|0.06344256387394694|0.23146321745512607|0.1602786646657306|0.015822683868241337|0.01845692143493136|0.005180491146119585|0.03341566660382575|0.010629477507206768|0.0|0.04968428130723066|0.029267958421589337|0.039312572856406824|0.0236655390376481|0.0|0.009286266604639564|0.0026304911589266535|0.0|0.0|0.0|0.0|0.0|0.0",
//                "0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.0|0.04641851361456718|0.026238199885605808|0.02765251769228965|0.004751830609593564|0.0043140141719051|0.0|0.0|0.011740171892622397|0.03135532563905452|0.09753067663314707|0.06981595613468942|0.005037344409279282|0.012942042515715301|9.540403043377993E-4|0.018793597283870297|0.007829934202359728|0.03798613497273225|0.1854487855975142|0.054491381850250625|0.0|0.0|6.552950176991383E-4|0.029465524200662285|0.017418441837965117"
//        };
//        app.associateString(imageA,imageBfeatureString);    // -- SURF !!!
//    }


    //----------------------------------------------------------------------------------------------

//
//    public void associateRgb( BufferedImage imageA , BufferedImage imageB ) {
//        Class imageRgbType = Planar.class;
//        Planar<GrayF32> inputAPlanar = ConvertBufferedImage.convertFromPlanar(imageA, null, true, GrayF32.class);   //   static <T extends ImageGray<T>> T 	convertFromSingle(BufferedImage src, T dst, Class<T> type)
//        Planar<GrayF32> inputBPlanar = ConvertBufferedImage.convertFromPlanar(imageB, null, true, GrayF32.class);
//
//        // stores the location of detected interest points
//        pointsA = new ArrayList<>();
//        pointsB = new ArrayList<>();
//
//        // stores the description of detected interest points
//        FastQueue<TD> descA = UtilFeature.createQueue(detDesc,100);
//        FastQueue<TD> descB = UtilFeature.createQueue(detDesc,100);
//
//        // describe each image using interest points
//        describeImage(inputAPlanar,pointsA,descA);
//        describeImage(inputBPlanar,pointsB,descB);
//
//        // Associate features between the two images
//        associate.setSource(descA);
//        associate.setDestination(descB);
//        associate.associate();
//
//        // display the results
//        AssociationPanel panel = new AssociationPanel(20);
//        panel.setAssociation(pointsA,pointsB,associate.getMatches());
//        panel.setImages(imageA,imageB);
//
//        ShowImages.showWindow(panel,"Associated Features",true);
//    }

//    /**
//     * Detect and associate point features in the two images.  Display the results.
//     */
//    public void associate( BufferedImage imageA , BufferedImage imageB )
//    {
//        T inputA = ConvertBufferedImage.convertFromSingle(imageA, null, imageType);   //   static <T extends ImageGray<T>> T 	convertFromSingle(BufferedImage src, T dst, Class<T> type)
//        T inputB = ConvertBufferedImage.convertFromSingle(imageB, null, imageType);
//
//        // stores the location of detected interest points
//        pointsA = new ArrayList<>();
//        pointsB = new ArrayList<>();
//
//        // stores the description of detected interest points
//        FastQueue<TD> descA = UtilFeature.createQueue(detDesc,100);
//        FastQueue<TD> descB = UtilFeature.createQueue(detDesc,100);
//
//        // describe each image using interest points
//        describeImage(inputA,pointsA,descA);
//        describeImage(inputB,pointsB,descB);
//
//        // Associate features between the two images
//        associate.setSource(descA);
//        associate.setDestination(descB);
//        associate.associate();
//
//        // display the results
//        AssociationPanel panel = new AssociationPanel(20);
//        panel.setAssociation(pointsA,pointsB,associate.getMatches());
//        panel.setImages(imageA,imageB);
//
//        ShowImages.showWindow(panel,"Associated Features",true);
//    }



    //----------------------------------------------------------------------------------------------




    /**
     * Detects features inside the two images and computes descriptions at those points.
     */
    private void describeImage(Planar<GrayF32> input, List<Point2D_F64> points, FastQueue<TD> descs )
    {
        /*		detDesc.detect(input);  // BREAKS HERE: INTERFACE REQUIRES <T> but I have a concrete class

		        for( int i = 0; i < detDesc.getNumberOfFeatures(); i++ ) {
			        points.add( detDesc.getLocation(i).copy() );
			        descs.grow().setTo(detDesc.getDescription(i));
		        }
        */
    }

/*
	//  https://searchcode.com/codesearch/view/16561132/
	private AssociateSurfBasic createAlg() {

		ScoreAssociation<TupleDesc_F64> score = new ScoreAssociateEuclidean_F64();
		AssociateDescription<TupleDesc_F64> assoc = FactoryAssociation.greedy(score, 20, true);

		return new AssociateSurfBasic(assoc);
	}


	public void associateSurfBasic( BufferedImage imageA , BufferedImage imageB )
	{
		T inputA = ConvertBufferedImage.convertFromSingle(imageA, null, imageType);   //   static <T extends ImageGray<T>> T 	convertFromSingle(BufferedImage src, T dst, Class<T> type)
		T inputB = ConvertBufferedImage.convertFromSingle(imageB, null, imageType);

		// stores the location of detected interest points
		pointsA = new ArrayList<>();
		pointsB = new ArrayList<>();

		// stores the description of detected interest points
		FastQueue<SurfFeature> descA = new FastQueue<SurfFeature>(100,SurfFeature.class,false);
		FastQueue<SurfFeature> descB = new FastQueue<SurfFeature>(100,SurfFeature.class,false);

		// describe each image using interest points
		describeImage(inputA,pointsA,descA);
		describeImage(inputB,pointsB,descB);

        AssociateSurfBasic associateSurfBasic = createAlg();
		// Associate features between the two images
		associateSurfBasic.setSource(descA);
		associateSurfBasic.setDestination(descB);
		associateSurfBasic.associate();

		// display the results
		AssociationPanel panel = new AssociationPanel(20);
		panel.setAssociation(pointsA,pointsB,associateSurfBasic.getMatches());
		panel.setImages(imageA,imageB);

		ShowImages.showWindow(panel,"Associated Features",true);
	}
*/
}
