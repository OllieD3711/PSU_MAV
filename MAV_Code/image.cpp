#include "image.h"

Image::Image(int w, int h){
	
	width=w;
	height=h;

	theta = 53;     
	phi = 40;  
	
	cap.set(CV_CAP_PROP_FORMAT,CV_8UC3);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
	
	if(!cap.open()){
		cout<<"Could not open camera.";
		while(1);
	}
	
        tf = tag36h11_create();
        td = apriltag_detector_create();
        apriltag_detector_add_family(td,tf);

	capThreadActive=false;
	boundThreadActive=false;
	mainThreadActive=false;

	getBoundary=false;
	getHomeBase=false;
	getTarget=false;
	getPickup=false;

}

void Image::startProcessing(){
	if(mainThreadActive==false){
		mainThreadActive=true;
		mainThread=std::thread(&Image::process,this);
	}
	#ifdef DISPLAY_MESSAGES
	cout<<"\nSpawning thread to process images"<<std::flush;
	#endif
}


void Image::stopProcessing(){
	if(mainThreadActive){
		mainThreadActive=false;
		if(mainThread.joinable()){
			mainThread.join();
			#ifdef DISPLAY_MESSAGES
			cout<<"\nJoined thread that processed images."<<std::flush;
			#endif
                        apriltag_detector_destroy(td);
                        tag36h11_destroy(tf);
		}
	}
}

void Image::startCapture(){
	if(capThreadActive==false){
		capThreadActive=true;
		capThread=std::thread(&Image::capture,this);
	}
	#ifdef DISPLAY_MESSAGES
	cout<<"\nSpawning thread to capture images"<<std::flush;
	#endif
}


void Image::stopCapture(){
	if(capThreadActive){
		capThreadActive=false;
		if(capThread.joinable()){
			capThread.join();
			#ifdef DISPLAY_MESSAGES
			cout<<"\nJoined thread that captured images."<<std::flush;
			#endif
		}
	}
}
		
void Image::capture(){
	while(capThreadActive){
		if(cap.grab()){
			Mat img;
			cap.retrieve(img);
			if(!buffers.frames.push(img)){
				#ifdef DISPLAY_MESSAGES
				cout<<"\nDropping a frame from image buffer.";
				#endif
			}
		}
	}
}	


void Image::calibrate(Scalar &u, Scalar &l)
{

	Mat image,hsv,thresh;

	cv::namedWindow("Test",cv::WINDOW_NORMAL);	
	resizeWindow("Test",640,480);

	while(!cap.grab());
	cap.retrieve(image);

	imshow( "Test", image );// show our image inside it.
	bitwise_not( image, image );
	

	int hueL=0,hueH=179,satL=0,satH=255,valL=0,valH=255;
	
	cv::createTrackbar("Lower Hue","Test",&hueL,179);
	cv::createTrackbar("Higher Hue","Test",&hueH,179);
	cv::createTrackbar("Lower Sat","Test",&satL,255);
	cv::createTrackbar("Higher Sat","Test",&satH,255);
	cv::createTrackbar("Lower Value","Test",&valL,255);
	cv::createTrackbar("Higher Value","Test",&valH,255);
	
	char ch='n';
	while(ch!='y')
	{
			while(!cap.grab());
			cap.retrieve(image);
	    		cvtColor(image, hsv, CV_BGR2HSV);
			inRange(hsv,cv::Scalar(hueL,satL,valL),cv::Scalar(hueH,satH,valH),thresh);
	    		imshow("Test",thresh);
	    		std::cout<<hueL<<" "<<hueH<<" "<<satL<<" "<<satH<<" "<<valL<<" "<<valH<<std::endl;
	    		cv::waitKey(50);
			//cin>>ch;
	}

	u=Scalar(hueH,satH,valH);
	l=Scalar(hueL,satL,valL);
}


void Image::setType(bool a, bool b, bool c, bool d){
	getBoundary=a;
	getHomeBase=b;
	getTarget=c;
	getPickup=d;
}

void Image::process(){
	Mat img, imgHSV, imgHB, imgB, imgT;// new blank image
	cv::Mat imgConcat;

	int dir;
	float dist;
	float x,y;

	//Starting taking frames from camera
	startCapture();
	uint64_t t1=getTimeUsec();
	int cnt=0;

	Scalar u,l;

	//calibrate(u,l);

	while(mainThreadActive){
		
		if(buffers.frames.pop(img)){

			imgData.homeBase.reset();
			imgData.boundary.reset();
			imgData.target.reset();			

			cvtColor(img, imgHSV, COLOR_BGR2HSV);
			
			if(getHomeBase){
				imgHB=detectHomeBase(imgHSV,buffers.messages.localPos.z,dir,x,y);
				imgData.homeBase.dir=dir;
				imgData.homeBase.x=x;
				imgData.homeBase.y=y;					
			}
			else{
				imgHB=Mat::zeros(Size(width,height),CV_8UC1);
			}

			if(getBoundary){
				imgB=detectBoundary(imgHSV, 1, dir,dist);
				
				imgData.boundary.dir=dir;
				imgData.boundary.dist=dist;
				
	
				char ch;
				switch(dir){
					case 1:ch='u';
					break;
	
					case 2:ch='r';
					break;
	
					case 3:ch='d';
					break;
	
					case 4:ch='l';
					break;
	
				}
				//cout<<"\nBoundary detected in "<<ch<<" direction at "<<dist<<" mts."<<std::flush;	
			}
			else{
				imgB=Mat::zeros(Size(width,height),CV_8UC1);
				cout<<"\nSize of bound img: "<<imgB.rows<<std::flush;
			}

			if(getTarget){
				imgT=detectTarget(img,buffers.messages.localPos.z,dir,x,y);
				imgData.target.dir=dir;					
			}
			else{
				imgT=Mat::zeros(Size(width,height),CV_8UC1);
			}
			
			
			//Setup concatenated image to send to ground station
			
			imgConcat=Mat::zeros(Size(width*3,height),CV_8UC1);

			imgHB.copyTo(imgConcat(Rect(0,0,width,height)));
			imgB.copyTo(imgConcat(Rect(width,0,width,height)));
			imgT.copyTo(imgConcat(Rect(width*2,0,width,height)));

			//imshow("g",imgConcat);
			//waitKey(50);
			
			buffers.sendFrame.push(imgConcat);
			
		}
	
		//if(getTimeUsec()-t1 >1000000){
		//	cout<<"\nFRAME RATE: "<<cnt;
		//	cnt=0;
		//	t1=getTimeUsec();
		//}
	}
	
	//Stop taking frames from camera
	stopCapture();
}

cv::Mat Image::detectTarget(cv::Mat imgHSV, float altitude, int &direction, float &x, float &y){

	Mat imgThresholded;

	//inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
	inRange(imgHSV, Scalar(0, 53, 130), Scalar(19, 162, 255), imgThresholded);

	//morphological opening (remove small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	//morphological closing (fill small holes in the foreground)
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	//Display image after applying thresholded ranges of Hue, Saturation, and Value
	//imshow("Thresholded Image", imgThresholded);

	//
	GaussianBlur(imgThresholded, imgThresholded, Size(9, 9), 2, 2);

	vector<Vec3f> circles;
	Point centre;
	//cout << imgThresholded.rows << endl;

	HoughCircles(imgThresholded, circles, CV_HOUGH_GRADIENT, 1, imgThresholded.rows/8, 200, 50, 0, 0);

	if(circles.size()==0)
		return imgThresholded;

	for(size_t i = 0; i < circles.size(); i++){
	    Point centre_record(cvRound(circles[i][0]), cvRound(circles[i][1]));
	    centre = centre_record;
	    int radius = cvRound(circles[i][2]);

	    circle(imgThresholded, centre, 3, Scalar(0, 0, 255), -1, 8, 0);

	    circle(imgThresholded, centre, radius, Scalar(0, 255, 0), 3, 8, 0);
	}

	  
	if(centre.x > 220 && centre.y > 80 && centre.y < 160 )
		{
			direction = 2;
			//cout<< "Right" << endl;
		}

	else if(centre.x < 140 && centre.y > 80 && centre.y < 160)
		{
			direction = 4;
			//cout<< "Left" << endl;
		}

	else if( centre.y > 160)
		{
			direction = 3;
			//cout<< "Back" << endl;
		}

	else if(centre.y > 0 && centre.y < 80)
		{
			direction = 1;
			//cout<< "Front" << endl;
		}

	else if(centre.x > 140 && centre.x < 220 && centre.y > 80 && centre.y < 160)
		{
			direction = 5;
			//cout<< "Drop" << endl;
		}
	
	//int theta = 53;        // Camera half-angle in x-direction (640 pixels)
 	//int phi = 40;          // Camera half-angle in y-direction (480 pixels)
	//int xMax = 640.0;      // Maximum x-axis grid location
	//int yMax = 480.0;      // Maximum y-axis grid location
	
	//int altitude = 1;
	//float x = centre.x * 2 * altitude / (sin(theta*3.14/180.0) * xMax);
	//float y = centre.y * 2 * altitude / (sin(phi*3.14/180.0) * yMax);
	
	//namedWindow("Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE);
	//imshow("Hough Circle Transform Demo", imgOriginal);
	//if(waitKey(30)>= 0) break;
	//waitKey(30);

	return imgThresholded;

}


cv::Mat Image::detectHomeBase(cv::Mat imgOriginal, float altitude, int &direction, float &x, float &y){

	cv::Mat imgThresholded;

	Size size(320,240);

	inRange(imgOriginal, Scalar(0, 53, 130), Scalar(19, 162, 255), imgThresholded);

	//morphological opening (remove small objects from the foreground)
	erode(imgOriginal, imgOriginal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgOriginal, imgOriginal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	//morphological closing (fill small holes in the foreground)
	dilate( imgOriginal, imgOriginal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgOriginal, imgOriginal, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	GaussianBlur(imgOriginal, imgOriginal, Size(9, 9), 2, 2);

	vector<Vec3f> circles;
	Point centre;
	
	//cout << imgThresholded.rows << endl;

	HoughCircles(imgOriginal, circles, CV_HOUGH_GRADIENT, 1, imgOriginal.rows/4, 200, 50, 0, 0);

	for(size_t i = 0; i < circles.size(); i++){
	    Point centre_record(cvRound(circles[i][0]), cvRound(circles[i][1]));
	    centre = centre_record;
	    int radius = cvRound(circles[i][2]);

	    circle(imgOriginal, centre, 3, Scalar(0, 0, 255), -1, 8, 0);

	    circle(imgOriginal, centre, radius, Scalar(0, 255, 0), 3, 8, 0);
	}

	  
	if(centre.x > 360 && centre.y > 200 && centre.y < 280)
		{
			direction = 2;
			//cout<< "Right" << endl;
		}

	else if(centre.x < 280 && centre.y > 200 && centre.y < 280)
		{
			direction = 4;
			//cout<< "Left" << endl;
		}

	else if( centre.y > 280)
		{
			direction = 3;
			//cout<< "Back" << endl;
		}

	else if(centre.y > 0 && centre.y < 200)
		{
			direction = 1;
			//cout<< "Front" << endl;
		}

	else if(centre.x > 280 && centre.x < 360 && centre.y > 200 && centre.y < 280)
		{
			direction = 5;
			//cout<< "Good" << endl;
		}
	
	int theta = 53;        // Camera half-angle in x-direction (640 pixels)
 	int phi = 40;          // Camera half-angle in y-direction (480 pixels)
	int xMax = 320.0;      // Maximum x-axis grid location
	int yMax = 240.0;      // Maximum y-axis grid location
	
	x = centre.x * 2 * altitude / (sin(theta*3.14/180.0) * xMax);
	y = centre.y * 2 * altitude / (sin(phi*3.14/180.0) * yMax);
	
	//cout << x <<","<< y << endl;
	
	//namedWindow("Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE);
	//imshow("Hough Circle Transform Demo", imgOriginal);
	//if(waitKey(30)>= 0) break;
	//waitKey();

	//float t_end = clock();
	//cout << (t_end - t_start)/(CLOCKS_PER_SEC)*1000 << endl;

	return imgThresholded;
}
	
cv::Mat Image::detectBoundary(cv::Mat imgHSV, float alt, int &direction, float &dist){

	cv::Mat imgInvHSV, binaryMat1, binaryMat2,dst, nonZeroCoordinates1; 

	//bitwise_not( imgHSV, imgInvHSV );

	//yellow
        //inRange(hsvImg1,Scalar(27,71,200),Scalar(179,255,255),binaryMat1); ////////////////// With carpet
	inRange(imgHSV,Scalar(0,181,60),Scalar(43,255,206),binaryMat1);
	
	//black
	//inRange(hsvImg2,Scalar(116,162,0),Scalar(179,255,255),binaryMat2); /////////////////// With carpet
	//inRange(imgInvHSV,Scalar(15,106,104),Scalar(83,255,207),binaryMat2);

	//bitwise_and(binaryMat1,binaryMat2,dst);
	
	findNonZero(binaryMat1, nonZeroCoordinates1);
	//cout<<"\nNZM "<< nonZeroCoordinates1.rows <<" "<<nonZeroCoordinates1.cols<< endl;

	if( nonZeroCoordinates1.rows == 0 || nonZeroCoordinates1.rows < 1000 )
		return Mat::zeros(Size(width,height),CV_8UC1);
				
	cv::Vec<float,4> scene_output;
	fitLine(nonZeroCoordinates1, scene_output, CV_DIST_L2, 0, 0.01, 0.01);

	float slope = scene_output[1]/scene_output[0] ;
	float y_intercept = scene_output[3] - slope * scene_output[2];
	float x_intercept = -(y_intercept/slope);

	///////////////////////////////////////////////////// distance to the boundary
	/* Compute shortest distance of quad from border */

	double x1, x2, x3, y1, y2, y3, len_a, len_b, len_c, area, s, r_pix;

	x1 = 0.0;
	x2 = width/2.0;
	x3 = x2;
	y2 = height/2.0;

	/* Compute y distance on border corresponding to an x location on the grid */
	y1 = slope*x1 + y_intercept;
	y3 = slope*x3 + y_intercept;

	/* Compute length of side of triangle given two grid points */
	len_a = sqrt(pow((x1-x2),2.0) + pow((y1-y2),2.0));
	len_b = sqrt(pow((x1-x3),2.0) + pow((y1-y3),2.0));
	len_c = sqrt(pow((x2-x3),2.0) + pow((y2-y3),2.0));

	s = 0.5*(len_a + len_b + len_c);
	area = sqrt(s*(s - len_a)*(s - len_b)*(s - len_c));
	r_pix = (2*area/len_b);
	dist = r_pix * 2 * alt / (sin(theta*3.14/180.0) * width);

	///////////////////////////////////////////////////// Uncomment for getting distance to the boundary

	//line( binaryMat,Point2f(0,y_intercept),Point2f(scene_output[2],scene_output[3]), Scalar( 255, 255, 255), 5, 8, 0);
	//line( binaryMat,Point2f(x_intercept,0),Point2f(scene_output[2],scene_output[3]), Scalar( 255, 255, 255), 5, 8, 0);

	///////////////////////////////////////////////////// Define the direction for the quad to go away from the boundary

	////// Up or Forward == 1
	////// Right == 2
	////// Down or Backward == 3
	////// Left == 4


	if(scene_output[2] > 320/2 && scene_output[3] > 240/2 && scene_output[3] < 360/2)
	{
		if(abs(slope) < 1)
		{
			direction = 1;
			//cout << "Front" << endl;
		}
		else
		{
			direction = 4;
			//cout << "Left" << endl;		
		}
	}
	else if(scene_output[2] > 320/2 && scene_output[3] > 360/2) 
	{
		direction = 1;
		//cout << "Front" << endl;
	}
	else if(scene_output[2] < 320/2 && scene_output[3] > 240/2 && scene_output[3] < 360/2) 
	{
		if(abs(slope) < 1)
		{
			direction = 1;
			//cout << "Front" << endl;
		}
		else
		{
			direction = 2;
			//cout << "Right" << endl;		
		}
	}
	else if(scene_output[2] < 320/2 && scene_output[3] > 360/2) 
	{
		direction = 1;
		//cout << "Front" << endl;
	}
	else if(scene_output[2] > 320/2 && scene_output[3] > 120/2 && scene_output[3] < 240/2) 
	{
		if(abs(slope) < 1)
		{
			direction = 3;
			//cout << "Down" << endl;
		}
		else
		{
			direction = 4;
			//cout << "Left" << endl;		
		}
	}
	else if(scene_output[2] > 320/2 && scene_output[3] < 120/2) 
	{
		direction = 3;
		//cout << "Down" << endl;
	}
	else if(scene_output[2] < 320/2 && scene_output[3] > 120/2 && scene_output[3] < 240/2) 
	{
		if(abs(slope) < 1)
		{
			direction = 3;
			//cout << "Down" << endl;
		}
		else
		{
			direction = 2;
			//cout << "Right" << endl;		
		}
	}
	else if(scene_output[2] < 320/2 && scene_output[3] < 120/2) 
	{
		direction = 3;
		//cout << "Down" << endl;
	}

	//	cout << direction << endl;
	//cout << "Slope : "<< slope << endl;
	//cout << "x0 : " << scene_output[2] << endl;
	//cout << "y0 : " << scene_output[3] << endl;

	return binaryMat1;
}


cv::Mat Image::detectApril(cv::Mat img, vector<tagPoseInfo> *tagLocs)
{
    /*// Create tag library to use
    apriltag_family_t *tf = NULL;
    tf = tag36h11_create();

    // Create tag detector object and assign the library
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td,tf);
    */

    // Set detector parameters
    /* quad_decimate - decimates/discard the input image by this factor
     *                  (1.0 uses entire input image)
     * quad_sigma - Adds gaussian blur to image
     * refine_edges - spends more time refining edges of tags
     * decode_sharpening
     */
    td->quad_decimate = 2.0;
    td->quad_sigma = 0.0;
    td->refine_edges = 1;
    td->decode_sharpening = 0.25;

    // Convert image to format which can be used by detector
    Mat gray;
    cvtColor(img,gray,COLOR_BGR2GRAY);

    // Make an image_u8_t header for the Mat data
    image_u8_t im = {.width = gray.cols,
                    .height = gray.rows,
                    .stride = gray.cols,
                    .buf = gray.data};

    // Detect april tags within image
    zarray_t *detections = apriltag_detector_detect(td,im);

    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++){
        apriltag_detection_t *det;
        zarray_get(detections,i,&det);

        // Draw tag square
        line(img, Point(det->p[0][0], det->p[0][1]),
                Point(det->p[1][0], det->p[1][1]),
                Scalar(0,0xff,0),2);
        line(img, Point(det->p[0][0], det->p[0][1]),
                Point(det->p[3][0], det->p[3][1]),
                Scalar(0,0,0xff),2);
        line(img, Point(det->p[1][0], det->p[1][1]),
                Point(det->p[2][0], det->p[2][1]),
                Scalar(0xff,0,0),2);
        line(img, Point(det->p[2][0], det->p[2][1]),
                 Point(det->p[3][0], det->p[3][1]),
                 Scalar(0xff, 0, 0), 2);

        // Calculate pose of tag
        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = APRILSIZE*0.001; // in meters
        info.fx = FOCALX;   // in pixels
        info.fy = FOCALY;   // in pixels
        info.cx = PRINCX;   // in pixels
        info.cy = PRINCY;   // in pixels
        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info,&pose);

        // Store all detected tags in array
        tagPoseInfo currentTag;
        currentTag.Info = det;
        currentTag.Pose = pose;
        tagLocs->push_back(currentTag);

        // Print tag id number to image
        stringstream ss;
        ss << det->id;
        String text = ss.str();
        int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontscale = 1.0;
        int baseline;
        Size textsize = getTextSize(text,fontface,fontscale,2,&baseline);
        putText(img, text,Point(det->c[0] - textsize.width/2,
                                det->c[1]+textsize.height/2),
                fontface, fontscale, Scalar(0xff, 0x99, 0), 2);

    }
    apriltag_detections_destroy(detections);

    return img;
}


void detTargetType(apriltag_detection_t *det, targetType *tt)
{
    int ID = det->id;
    if ((ID == 119) || (ID == 120) || (ID == 121) || (ID == 122))
        tt-isHelipad == true;
    else if ((ID == 123) || (ID == 124) || (ID == 125) || (ID == 126))
        tt->isFerry == true;
    else if ((ID == 127) || (ID == 128) || (ID == 129) || (ID == 130))
        tt->isBullseye\ == true;
    else if ((ID == 131) || (ID == 132) || (ID == 133) || (ID == 134))
        tt->isPickUp == true;
}
