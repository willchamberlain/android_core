package william.chamberlain.androidvosopencvros;

import android.support.annotation.NonNull;

/**
 * Data for one feature / one set of features.
 *
 * constructor(data streams) : hooks into other data streams
 *
 * fixed-maxCachedDataSize array(s) - one per data element : initially empty
 *
 * synchronized add(data): check for full array; if full, copy all-but-first to new array, add new data to end of array
 *
 */
class DataTrack {


    private DetectedFeature[] data;
    private int maxCachedDataSize = 20;     // TODO - magic number
    private int sizeNow;

    public int sizeNow() {
        return sizeNow;
    }

    public DataTrack(int size_) {
        maxCachedDataSize = size_;
        data = new DetectedFeature[size_];
        sizeNow=0;
    }

    public synchronized void add(DetectedFeature detectedFeature){  //  TODO: not efficient - look at circular lists / circular queues
        if(null == detectedFeature) {
System.out.println("DataTrack: add(null) :  !!! ERROR !!!");
        }
        if(sizeNow < maxCachedDataSize) {
System.out.println("DataTrack: add("+detectedFeature.featureCanonicalDescriptor()+"): sizeNow < maxCachedDataSize");
            sizeNow++;
            data[sizeNow-1]=detectedFeature;
        } else {
System.out.println("DataTrack: add("+detectedFeature.featureCanonicalDescriptor()+"): sizeNow >= maxCachedDataSize");
            DetectedFeature[] newData = new DetectedFeature[maxCachedDataSize];
            for (int i_ = 0; i_ < maxCachedDataSize - 1; i_++) {    //  e.g. if maxCachedDataSize = 10    1 ... 9
                data[i_] = data[i_ + 1];                            //  e.g. if maxCachedDataSize = 10    newData[0]=data[1] ... newData[8]=data[9]
                data[maxCachedDataSize - 1] = detectedFeature;      //  e.g. if maxCachedDataSize = 10    newData[9]=detectedFeature
            }
        }
    }

    /**
     *
     * @return
     */
    public DetectedFeature[] data() {
        if(sizeNow < maxCachedDataSize) {
            return subArray(0,sizeNow);
        } else {
            return data;
        }
    }
    public DetectedFeature[] data(int windowSize_) {
        if(sizeNow <= windowSize_) {
            if (sizeNow < maxCachedDataSize) {
                return subArray(0,windowSize_);
            } else {
                return data;
            }
        } else {  //  sizeNow > windowSize_  {
            return subArray(sizeNow-windowSize_, windowSize_);  // get the most recent subset  e.g. if cache max is 15, and has collected 12, and is asked for 5, windowSize_ = 5, sizeNow = 12, start_ = 12-5 = 7
        }
    }

    @NonNull
    private DetectedFeature[] subArray(int start_, int windowSize_) {
        DetectedFeature[] newData = new DetectedFeature[windowSize_];
        for (int i_ = 0; i_ < windowSize_; i_++) {                         //  e.g. if windowSize_ = 7    0 ... 6
            newData[i_] = data[i_+start_];                                 //  e.g. if start_ = 5, windowSize_ = 7   newData[0]=data[5] ... newData[6]=data[11]
        }
        return newData;
    }
}
