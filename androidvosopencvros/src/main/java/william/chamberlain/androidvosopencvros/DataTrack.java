package william.chamberlain.androidvosopencvros;

/**
 * Data for one feature / one set of features.
 *
 * constructor(data streams) : hooks into other data streams
 *
 * fixed-windowSize array(s) - one per data element : initially empty
 *
 * synchronized add(data): check for full array; if full, copy all-but-first to new array, add new data to end of array
 *
 */
class DataTrack {


    private DetectedFeature[] data;
    private int windowSize;
    private int sizeNow;

    public DataTrack(int size_) {
        windowSize = size_;
        data = new DetectedFeature[size_];
        sizeNow=0;
    }

    public synchronized void add(DetectedFeature detectedFeature){  //  TODO: not efficient - look at circular lists / circular queues
        if(sizeNow < windowSize) {
            sizeNow++;
            data[sizeNow-1]=detectedFeature;
        } else {
            DetectedFeature[] newData = new DetectedFeature[sizeNow];
            for (int i_ = 1; i_ < sizeNow; i_++) {       //  e.g. if windowSize = 10    1 ... 9
                newData[i_ - 1] = data[i_];              //  e.g. if windowSize = 10    newData[0]=data[1] ... newData[8]=data[9]
                newData[sizeNow-1] = detectedFeature;    //  e.g. if windowSize = 10    newData[9]=detectedFeature
                data = newData;
            }
        }
    }

    /**
     *
     * @return
     */
    public DetectedFeature[] data() {
        if(sizeNow < windowSize) {
            DetectedFeature[] newData = new DetectedFeature[sizeNow];
            for (int i_ = 0; i_ < sizeNow; i_++) {    //  e.g. if sizeNow = 7    0 ... 6
                newData[i_] = data[i_];              //  e.g. if sizeNow = 7    newData[0]=data[0] ... newData[6]=data[6]
            }
            return newData;
        } else {
            return data;
        }
    }
}
