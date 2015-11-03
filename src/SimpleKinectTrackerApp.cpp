#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/Surface.h"
#include "cinder/gl/Texture.h"
#include "cinder/params/Params.h"
#include "cinder/Surface.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"
#include "cinder/Rand.h"
#include <algorithm>

#include "cinder/ImageIo.h"

//To record movie
#include "cinder/qtime/MovieWriter.h"

//To play movie
#include "cinder/qtime/QuickTime.h"

//Provided by the Kinect block
#include "CinderFreenect.h"

//Provided by the OSC block
#include "OscSender.h"
#include "OscListener.h"

//OpenCV
#include "CinderOpenCv.h"

//Load/Save config from XML
#include "CinderConfig.h"

//#include <vector>
#include "Resources.h"


using namespace ci;
using namespace ci::app;
using namespace std;


#define WINDOW_SIZE 12
#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480
#define POI_DEPTH_THRESHOLD 0.8f
#define POI_TRIGGER_THRESHOLD 40
#define DEPTH_RESOLUTION 65535

inline uint32_t diff( uint32_t a, uint32_t b){
    return a > b ? a - b : b - a;
}

inline float diff( float a, float b){
    return a > b ? a - b : b - a;
}

typedef struct {
    float avg; //the average depth
    uint16_t minDepth; //the closest sampled depth
    uint16_t maxDepth; //the farthest sampled depth
    //uint32_t filteredDepth[DEPTH_WIDTH * DEPTH_HEIGHT];
} Stats;

/*
 Definition through OSC messages:
 poiID : string, pos.x : float (0..1), pos.y : float(0..1), radius: int
 */
typedef struct{
    std::string poiID;
    Vec2i pos;
    int32_t radius;
    uint16_t trigger;
} POI;

class SimpleKinectTrackerApp : public AppNative {
private:
    float computeChangeWindow(Stats statistics);
    template < typename T >
    Stats doStatistics(T *depthData);
    void resetPOIriggers();
    void parseOscMessages();
    void sendOSC_stats( float wndAvg );
    void sendOSC_trigger( POI poi );
    void sendOSC_blobs();
    bool loadMovie();
    void startRecording();
    void stopRecording();
    void boostChange();

    
public:
    void prepareSettings( Settings* settings );
    void setup();
    void shutdown();
    void mouseUp( MouseEvent event );
    void keyDown( KeyEvent event );
    void update();
    void draw();
    void saveConfig();
    void loadConfig();
    
    KinectRef		mKinect;
    gl::Texture		mColorTexture, mDepthTexture;

    //Pixel stats
    float previousAverageDepth = 0;
    float previousAverageChangeInDepth = 0;
    
    float * changeWindow[WINDOW_SIZE];
    int windowFillIndex = 0;
    
    //Interest points
    std::vector< POI > pointsOfInterest;
    
    //OSC
    int oscOutPort = 3000;
    int oscInPort = 3001;
    std::string host = "localhost";
    osc::Sender oscSender;
    osc::Listener oscListener;
    
    qtime::MovieWriterRef	mMovieWriter;

    bool recordDepth = false;
    bool movieMode = false;
    qtime::MovieSurfaceRef	mMovie;
    
    Surface			mSurface;
    bool activeFrame = false;
    
    typedef vector< vector<cv::Point> > ContourVector;
    ContourVector mContours;
    //Blob detection
    int mStepSize;
    int mBlurAmount;
    //Contrast/brigthness
    double alpha;
    int beta;
    double threshold;
    
    int blobCount;
    
    cv::Mat backgroundReference;
    
    
    params::InterfaceGlRef mParams;
    config::ConfigRef     mConfig;
    string	configFilename;
    
    bool snapNextFrame = true;
    
    
    
};

void SimpleKinectTrackerApp::prepareSettings( Settings* settings )
{
    settings->setWindowSize( 1280, 480 );
}

void SimpleKinectTrackerApp::setup()
{
    
     mStepSize = 52;
     mBlurAmount = 11;
     alpha = 2.2;
     beta = 59;
     threshold = 20;
     blobCount = 0;
     
    configFilename = "config.xml";
    mParams = params::InterfaceGl::create( "Settings", Vec2i( 200, 100 ) );
    mConfig = config::Config::create(mParams);
    mConfig->addParam( "Threshold Step Size", &mStepSize).min(1).max( 255 ).step(1.0f);
    mConfig->addParam( "CV Blur amount", &mBlurAmount).min(3).max(55).step(1.0f);
    mConfig->addParam( "Contrast", &alpha).min(1.0f).max(3.0f).step(0.1f);
    mConfig->addParam( "Brightness", &beta).min(0).max(255).step(1.0f);
    mConfig->addParam( "Threshold", &threshold).min(0).max(255).step(1.0f);
    mParams->addParam( "Blob", &blobCount);

    console() <<getAppPath() <<std::endl;
    if(fs::exists( getAppPath() / fs::path(configFilename) ))
        loadConfig();
    
    pointsOfInterest.clear();
    oscSender.setup( host, oscOutPort, true);
    
    oscListener.setup( oscInPort );
    console() << "There are " << Kinect::getNumDevices() << " Kinects connected." << std::endl;
    
    if(  Kinect::getNumDevices() > 0 )
        mKinect = Kinect::create();
    else{
        movieMode = loadMovie();
        if(!movieMode)
            exit(0);
    }
}

void SimpleKinectTrackerApp::shutdown()
{
    saveConfig();
}

void SimpleKinectTrackerApp::saveConfig()
{
    mConfig->save( getAppPath() / fs::path(configFilename) );
}

void SimpleKinectTrackerApp::loadConfig()
{
    mConfig->load( getAppPath() / fs::path(configFilename) );
}

void SimpleKinectTrackerApp::mouseUp( MouseEvent event )
{
  if( event.getX() >= DEPTH_WIDTH )
      return;
    
    //pointsOfInterest.push_back( {"poi", event.getPos(), 20, 0}  );
}

void SimpleKinectTrackerApp::keyDown( KeyEvent event )
{
    if( event.getCode() == KeyEvent::KEY_SPACE ){
        if( recordDepth )
            stopRecording();
        else
            startRecording();
    }else if( event.getChar() == 'b' ){
        snapNextFrame = true;
    }
}

void SimpleKinectTrackerApp::parseOscMessages(){
    osc::Message message;
    while( oscListener.hasWaitingMessages() ) {

        oscListener.getNextMessage( &message );
        
        if( message.getAddress() == "/mimodek/trigger/add"){ //add a point of interest
            if( message.getArgType(0) == osc::TYPE_STRING &&
                message.getArgType(1) == osc::TYPE_FLOAT &&
                message.getArgType(2) == osc::TYPE_FLOAT &&
               message.getArgType(3) == osc::TYPE_INT32){
                pointsOfInterest.push_back( {
                    message.getArgAsString(0),
                    Vec2i( round(message.getArgAsFloat(1) *  DEPTH_WIDTH), round(message.getArgAsFloat(2) *  DEPTH_HEIGHT)),
                    message.getArgAsInt32(3),
                    0
                }  );
            }else{
                console() << "OSC: Wrong date types for /mimodek/trigger/add" << std::endl;
            }
            continue;
        }
        
        console() << "OSC: unknowb message received, address: " << message.getAddress() << std::endl;
    }
}

void SimpleKinectTrackerApp::sendOSC_stats( float wndAvg ){
//    console() << wndAvg * 100.0 << std::endl;
    if( wndAvg * 100.0 < 0.8 )
        return;
    osc::Message message;
    message.setAddress("/mimodek/activity/");
    message.addFloatArg( wndAvg );
    oscSender.sendMessage(message);
}

void SimpleKinectTrackerApp::sendOSC_trigger( POI poi ){
    osc::Message message;
    message.setAddress("/mimodek/trigger/fire");
    message.addStringArg(poi.poiID);
    oscSender.sendMessage(message);
}

void SimpleKinectTrackerApp::sendOSC_blobs(){
    //Send blobs count
    //Send vertical position of closest one
    if(mContours.size() <= 1){
        console() << "Skiping this frame" << std::endl;
        return; //no need to spam the receiver
    }
    
    float farthestY = DEPTH_HEIGHT + 1.0f;
    for( ContourVector::iterator iter = mContours.begin(); iter != mContours.end(); ++iter )
    {
        
        for( vector<cv::Point>::iterator pt = iter->begin(); pt != iter->end(); ++pt ){
            if(fromOcv(*pt).y < 2)
                continue;
            farthestY = fromOcv(*pt).y > farthestY ? farthestY : fromOcv(*pt).y;
        }
    }
    
    farthestY /= DEPTH_HEIGHT;
    
    osc::Message message;
    message.setAddress("/mimodek/blobs/");
    message.addIntArg(mContours.size() - 1);
    message.addFloatArg( farthestY );
    oscSender.sendMessage(message);
    
            
//    console() << "Blob count: " << mContours.size() << ", farthestY: " << farthestY << std::endl;
}

template <typename T>
Stats SimpleKinectTrackerApp::doStatistics(T *depthPixels){
    
    //std::shared_ptr<uint16_t> depthPixels = mKinect->getDepthData(); //Holds the depth data from the Kinect
    Stats statistics = { 0, DEPTH_RESOLUTION, 0};
    T depthPixel;
    size_t x,y;
    for( size_t p = 0; p < DEPTH_WIDTH * DEPTH_HEIGHT; ++p ) {
        depthPixel = depthPixels[p];
        statistics.avg += depthPixel > previousAverageDepth ? 2 * depthPixel : depthPixel;
        statistics.minDepth = statistics.minDepth > depthPixel ? depthPixel :  statistics.minDepth;
        statistics.maxDepth = statistics.maxDepth < depthPixel ? depthPixel :  statistics.maxDepth;
        
        if( ((float)depthPixel / (float)DEPTH_RESOLUTION) > POI_DEPTH_THRESHOLD  ){
            //Trigger relevant POI
            y = p / DEPTH_WIDTH;
            x = p - (y * DEPTH_WIDTH);
            Vec2i pixPos = Vec2i( x, y );
            
            for (std::vector<POI>::iterator it = pointsOfInterest.begin() ; it != pointsOfInterest.end(); ++it){
                if( (*it).trigger >= POI_TRIGGER_THRESHOLD || (*it).pos.distanceSquared( pixPos ) > (*it).radius * (*it).radius) //square of the radius
                    continue;
               
                (*it).trigger++;
            }
        }
        
    }
    statistics.avg /= (float)(DEPTH_WIDTH * DEPTH_HEIGHT);
    return statistics;
}

void SimpleKinectTrackerApp::resetPOIriggers(){
    for (std::vector<POI>::iterator it = pointsOfInterest.begin() ; it != pointsOfInterest.end(); ++it){
        (*it).trigger = 0;
    }
}

float SimpleKinectTrackerApp::computeChangeWindow( Stats statistics){
   
    
    if( windowFillIndex == WINDOW_SIZE ){
        //The window is full, add one element at the end, remove the first, push the rest
        changeWindow[0] = changeWindow[1];
        *changeWindow[WINDOW_SIZE - 1] = diff(previousAverageDepth, statistics.avg);
    }else{
        //Fill up the window
        changeWindow[windowFillIndex] = (float *)malloc(4);
        *changeWindow[windowFillIndex] = diff(previousAverageDepth, statistics.avg);
        windowFillIndex++;
    }
    
    float windowAvg = 0;
    
    for(int i = 0; i < windowFillIndex; i++){
        windowAvg += *changeWindow[i];
    }
    
    windowAvg /= (float)windowFillIndex;
    
    
    float instantChange = diff(previousAverageChangeInDepth, windowAvg);
    previousAverageChangeInDepth = windowAvg;
    previousAverageDepth = statistics.avg;
    
    return instantChange;

}

void SimpleKinectTrackerApp::startRecording(){
    fs::path path = getSaveFilePath();
    if( path.empty() )
        return; // user cancelled save
    
    // The preview image below is entitled "Lava" by "Z T Jackson"
    // http://www.flickr.com/photos/ztjackson/3241111818/
    
    qtime::MovieWriter::Format format;
    if( qtime::MovieWriter::getUserCompressionSettings( &format, loadImage( loadResource( RES_PREVIEW_IMAGE ) ) ) ) {
        mMovieWriter = qtime::MovieWriter::create( path, DEPTH_WIDTH, DEPTH_HEIGHT, format );
    }
    
    recordDepth = true;

}

void SimpleKinectTrackerApp::stopRecording(){

    recordDepth = false;
    mMovieWriter->finish();

}

bool SimpleKinectTrackerApp::loadMovie(){
    fs::path moviePath = getOpenFilePath();
    if( moviePath.empty() )
        return false;

        try {
            // load up the movie, set it to loop, and begin playing
            mMovie = qtime::MovieSurface::create( moviePath );
            
            console() << "Dimensions:" << mMovie->getWidth() << " x " << mMovie->getHeight() << std::endl;
            console() << "Duration:  " << mMovie->getDuration() << " seconds" << std::endl;
            console() << "Frames:    " << mMovie->getNumFrames() << std::endl;
            console() << "Framerate: " << mMovie->getFramerate() << std::endl;
            console() << "Has audio: " << mMovie->hasAudio() << " Has visuals: " << mMovie->hasVisuals() << std::endl;
            
            mMovie->setLoop( true, true );
            mMovie->setRate( 0.5f );
            mMovie->seekToStart();
            mMovie->play();
        }
    catch( ... ) {
        console() << "Unable to load the movie." << std::endl;
        mMovie->reset();
        return false;
    }
    
    mDepthTexture.reset();
    return true;
}


void SimpleKinectTrackerApp::update()
{
    resetPOIriggers();
    bool processFrame = false;
    Stats frameStats;
    cv::Mat gray;
    cv::Mat thresh;
    
    if( movieMode && mMovie ){
        mSurface = mMovie->getSurface();
        //frameStats = doStatistics( mSurface.getData() );
        mDepthTexture = gl::Texture( mSurface );
        cv::Mat input( toOcv( mSurface ) );
        cv::cvtColor( input, input, CV_RGB2GRAY );
        cv::threshold(input, gray, threshold, 255, cv::THRESH_BINARY);
        
        processFrame = true;
    }else if( !movieMode && mKinect->checkNewDepthFrame() ){

        mDepthTexture = mKinect->getDepthImage();

        mSurface = mKinect->getDepthImage();
        cv::Mat input( toOcv( Channel8u( mSurface )  ) );
        cv::threshold(input, gray, threshold, 255, cv::THRESH_BINARY);
        //frameStats = doStatistics( mKinect->getDepthData().get() );

        processFrame = true;
        
        if( mKinect->checkNewVideoFrame() )
            mColorTexture = mKinect->getVideoImage();
    }
    
    if(processFrame){
     
        /*
        if(snapNextFrame){
            backgroundReference = cv::Mat(toOcv( mSurface ));
            snapNextFrame = false;
            return;
        }
        */
        
        //cv::Mat input( toOcv( mSurface ) );
       

        /*
        cv::Mat scaled;
        cv::Mat scaledGray;
        
        if(input.empty()){
            return;
        }
        

        input.convertTo(input,CV_32F);
        
        if(snapNextFrame){
            backgroundReference = cv::Mat(toOcv( mSurface ));
            backgroundReference.convertTo(backgroundReference,CV_32F);
            snapNextFrame = false;

        }
        
        if(false && snapNextFrame){
            backgroundReference = gray;
            snapNextFrame = false;
        }
        */
        
        //gray = input - backgroundReference;
        //backgroundReference = cv::Mat(input);
        
        //cv::equalizeHist(gray, gray);

        gray.convertTo(gray, -1, alpha, beta);
        cv::blur( gray, gray, cv::Size( mBlurAmount, mBlurAmount ) );

        
        
//       cv::accumulateWeighted(input, backgroundReference, 0.320 );
        //cv::convertScaleAbs(backgroundReference, scaled);
        //cv::cvtColor( input, gray, CV_RGB2GRAY );
        //cv::cvtColor( scaled, scaledGray, CV_RGB2GRAY );
        //diff = input - scaled;
        
        //cv::cvtColor( diff, diff, CV_RGB2GRAY );


        //cv::convertScale( brackgroundReference, running_average_in_display_color_depth, 1.0, 0.0 )


       // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(/*clipLimit=2.0, tileGridSize=(8,8)*/);
        

        


        
        
                //clahe->apply(gray, gray);
//        saturate_cast<uchar>( alpha*( image.at<Vec3b>(y,x)[c] ) + beta )
        

        

        //cv::fastNlMeansDenoising(gray, gray,10,7,21);
        mContours.clear();
        mColorTexture = gl::Texture( fromOcv( gray ) );
        
        
        for( int t = 0; t <= 255; t += mStepSize )
        {
            ContourVector vec;
            cv::threshold( gray, thresh, t, 255, CV_8U );
            //thresh.convertTo(input,CV_32F);
            cv::findContours( thresh, vec, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
            // put into mContours
            mContours.insert( mContours.end(), vec.begin(), vec.end() );
        }
        
        blobCount = mContours.size();
        /*
         console() << "Frame avg: " << frameStats.avg;
         console() << ", min: " << frameStats.minDepth;
        console() << ", max: " << frameStats.maxDepth << std::endl;
         */
         
        //sendOSC_stats( computeChangeWindow(frameStats) );
        sendOSC_blobs( );
    }

    
    //Hanfle incoming OSC messages
    parseOscMessages();
    
    //	console() << "Accel: " << mKinect.getAccel() << std::endl;
}

void SimpleKinectTrackerApp::draw()
{
    gl::clear( Color( 0, 0, 0 ) );
    gl::color(1, 1, 1 );
    gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );
    if( mDepthTexture )
        gl::draw( mDepthTexture );
    if( mColorTexture )
        gl::draw( mColorTexture, Vec2i( 640, 0 ) );
    
    /*
    if( activeFrame ){
        gl::color(0, 1, 0, 0.3 );
        gl::drawSolidRect( Rectf(0,0, DEPTH_WIDTH, DEPTH_HEIGHT ) );
    }
     */
    
    for (std::vector<POI>::iterator it = pointsOfInterest.begin() ; it != pointsOfInterest.end(); ++it){
        if( (*it).trigger >= POI_TRIGGER_THRESHOLD  ){
            gl::color(0, 1, 0, 1 ); // 3 * r^2
            sendOSC_trigger( (*it) );
        }else{
            gl::color(1, 1, 1, 0.5 ); // 3 * r^2
        }
        gl::drawSolidCircle( (*it).pos, (*it).radius);

    }
    
    // add this frame to our movie
    if( recordDepth && mMovieWriter )
        mMovieWriter->addFrame( copyWindowSurface( Area( Vec2i(0,0), Vec2i( DEPTH_WIDTH, DEPTH_HEIGHT )) ) );
    
    // draw the contours
    for( ContourVector::iterator iter = mContours.begin(); iter != mContours.end(); ++iter )
    {
        glBegin( GL_LINE_LOOP );
        
        for( vector<cv::Point>::iterator pt = iter->begin(); pt != iter->end(); ++pt )
        {
            gl::color( Color( 1.0f, 0.0f, 0.0f ) );
            gl::vertex( fromOcv( *pt ) );
        }
        
        glEnd();
    }
    
    // draw interface
    mParams->draw();
}


CINDER_APP_NATIVE( SimpleKinectTrackerApp, RendererGl )