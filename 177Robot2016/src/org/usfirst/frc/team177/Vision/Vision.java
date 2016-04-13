package org.usfirst.frc.team177.Vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ni.vision.VisionException;

import edu.wpi.first.wpilibj.CameraServer;


public class Vision {
	
	//used to prevent crashing if loaded on a controller without OpenCV 
	private boolean openCVisGood = true;
	
	//Change this to false to disable looking for the reference triangle.
	private static final boolean useReferenceTarget = false;
	
	private static final int referenceMinPixel = 200; // don't look for the reference above this point.
	private static final double minReferenceWidth = 20;
	private static final double referenceExpectedAspect = 1.7;
	private static final double referenceAspectTheshold = 0.5;
	private static final int defaultReferenceTargetOffset = 8; //tweak this on robot 
	
	//change this to false to disable doing the polygon approximation
	private static final boolean usePolygonApproximation = true;
	
	public List<MatOfPoint> referenceTarget = new ArrayList<MatOfPoint>(); //for updating image
	int referenceTargetOffset = 0;	
	
	static final double minTargetWidth = 20;		
	static final double expectedAspect = 5.0/3.0;
	static final double aspectTheshold = 0.3;
	static final int cameraIndex	= 0; //should be 0 on robot?
	
	public final double BAD_BEARING = 999;
	
	Mat refFeatures;
	
	CameraServer server; //Used to show image on smart dashboard.
	Image dashImage;
	Mat lastCapture;
	long lastCaptureTime;
	long lastProccessTime;
	double bearing;
	
	Mat saveImage;
	 
	private final CaptureThread captureThread;
	//private final ProcessThread proccessThread;
	private final DSThread dsThread;
	
	private Object lastCaptureLock = new Object();
	private Object saveImageLock = new Object();
	
	VideoCapture videoCapture;

	public Vision()
	{
		//load the opencv libraries
		try {
			System.load("/usr/local/lib/libopencv_java310.so");
		} catch (UnsatisfiedLinkError  e) {
			System.out.println( "Unable to load openCV library, no vision for you!\n" + e);
			openCVisGood = false;
		}				
		
        //start the camera server to show the image on the dashboard
	    server = CameraServer.getInstance();
        server.setQuality(50);

		
		if(openCVisGood)
		{	
	    	videoCapture = new VideoCapture();
	    	
			captureThread = new CaptureThread();
			//  proccessThread = new ProcessThread();
			dsThread = new DSThread();

			dashImage = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
			
			//values from test program.
			 double[] m = new double[]{0.1519652270416018,
			 -0.06763131030219712,
			 2.22955847505135,
			 2.432277662748807,
			 4.759803166789117,
			 2.117252961943986,
			 3.858388579534699};
			refFeatures = new Mat(7,1,6);
			
			for (int i=0; i<7; i++)
			{ 
				refFeatures.put(i, 0, m[i]);
			}
			
			//System.out.println("Reference Hu:" + refFeatures.dump());
			
			//setup double buffering
		    lastCapture = new Mat();
		    saveImage = new Mat();	       
		}			       
		else
		{
			//if there is no opencv, try sending the camera directly to the DS
	        //the camera name (ex "cam0") can be found through the roborio web interface
	        server.startAutomaticCapture("cam5"); //changed to cam5 on PB
	        
	        //Initialize unused threads to null to keep compiler happy
	        captureThread = null;
	        dsThread = null;
		}
	}
	
	Mat log10(Mat v) {
		for (int i = 0; i < v.rows(); ++i) {
			double [] col = v.get(i,  0);
			for(int j = 0; j < v.cols(); j++) {
				col[j] = Math.log10(Math.abs(col[j]));
			}
			v.put(i, 0, col);
		}
		return v;
	}
	
	double arrayDelta(Mat a, Mat b) {
		double accum = 0;
		for (int i = 0; i < a.rows(); ++i) {
			double [] cola = a.get(i,  0);
			double [] colb = b.get(i,  0);
			for(int j = 0; j < a.cols(); j++) {
				accum += (cola[j] - colb[j]);
			}			
		}
		return accum;
	}		
	
	public double getBearing() {
		return getBearing(false);
	}
	
	public double getBearing(boolean force) {		
		
		if (!openCVisGood) return BAD_BEARING;
		
		//If the analysis isn't current update.
		if (force || (System.nanoTime() - lastProccessTime)/1000000 > 66)
		{
			UpdateAnalysis(false);
		}
		
		return bearing;
	}
	
	
	/* Disabled for now, I think this can be an as need rather than allways running.
	private class ProcessThread implements Runnable {
		Mat tempImage;
		
		public ProcessThread() {
			new Thread(this).start();
			tempImage = new Mat();
		}
		
		public void run() {  
			
			if(!openCVisGood) return;
			
			long startTime;
			
			boolean updateFile = false;
					
			while (true) {
				startTime = System.nanoTime();

				UpdateAnalysis(false);
				try {
					long runTime = (System.nanoTime() - startTime)/1000000;
					//System.out.println("Process Thread: " + runTime);
					if(runTime < 1000) {
						Thread.sleep(1000 - runTime); //Process at 1Hz
					}
				} catch (InterruptedException e) {
					System.out.println("InterruptedException: " + e);
				}
			}
		}		
	}*/
	

	boolean UpdateAnalysis(boolean saveImage)
	{
		if(!openCVisGood) return false;
		
		Mat tempImage = new Mat();
		if(lastCapture != null && !lastCapture.empty()) {
			synchronized(lastCaptureLock){
				lastCapture.copyTo(tempImage);
			}
			
			lastProccessTime = System.nanoTime();
			return AnalyzeTarget(tempImage, saveImage);
		}
		else
		{
			return false;  // no capture
		}
	}
	
	Rect findTarget(Mat Image)
	{
		if (!openCVisGood) return null;
		if (Image == null) return null;
		
		Mat IMask = new Mat();	
		MatOfPoint2f approx = new MatOfPoint2f();
		MatOfPoint2f contour = new MatOfPoint2f();
		
		List<Rect> interestingBlobs = new ArrayList<Rect>();		
		List<MatOfPoint> interestingReferenceBlobs = new ArrayList<MatOfPoint>();	
							
		// Only retain pixels where G > 100		
		Core.inRange(Image, new Scalar(0, 150, 0), new Scalar(255, 255, 255), IMask);
	
		// Convert to HSV and apply threshold		
		/*Mat hsv = new Mat();
		Imgproc.medianBlur(image, image, 3);
		Imgproc.cvtColor(image, hsv, Imgproc.COLOR_BGR2HSV);
		Core.inRange(hsv, new Scalar(0, 0, 230), new Scalar(255, 255, 255), IMask);
	 	*/
		
		// Find contours to identify blobs
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();		 
		Imgproc.findContours(IMask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE, new Point(0, 0));
		
		// Loop through each contour to find ones within tolerance for correct Aspect Ratio		
		for (int i = 0; i < contours.size(); ++i) {
			Rect BB = Imgproc.boundingRect(contours.get(i));
			double aspectRatio = BB.width / (double) BB.height;

			if (BB.width >= minTargetWidth && Math.abs(aspectRatio - expectedAspect) <= aspectTheshold) {
				if(usePolygonApproximation)
				{
					//only add shapes with 8 sides
					contours.get(i).convertTo(contour, CvType.CV_32FC2);
					Imgproc.approxPolyDP(contour, approx, 3, true);			
					if(approx.rows() >= 7 && approx.rows() <= 9 )
					{			
						interestingBlobs.add(BB);
					}	
					/*else
					{
						System.out.println("Rejecting: " + BB.x + "x" + BB.y + " " + approx.rows());
					}*/
				} else {
					interestingBlobs.add(BB);
				}					
			}
			
			if(useReferenceTarget && BB.width >= minReferenceWidth && BB.y > referenceMinPixel && Math.abs(aspectRatio - referenceExpectedAspect) <= referenceAspectTheshold) {
				interestingReferenceBlobs.add(contours.get(i));
			}
		}
		
		//find the most interesting blob in the world
		int minIdx = -1;
		double minValue = 100000;
		for (int i = 0; i < interestingBlobs.size(); ++i) {
			Rect BB = interestingBlobs.get(i);
			Mat Icrop = IMask.submat(BB);

			// Compute the Hu Moments
			Mat hu = new Mat();
			Imgproc.HuMoments(Imgproc.moments(Icrop, true), hu);
						
			hu = log10(hu);
			
			double delta = Math.abs(arrayDelta(hu, refFeatures));

			if (delta < minValue) {
				minIdx = i;
				minValue = delta;
			}
		}
		
		if(useReferenceTarget)
		{
			//try to find the reference by finding the largest triangle		
			int maxReferenceIdx = -1;
			double maxReferenceValue = 0;		
			
			MatOfPoint2f maxReferenceTarget = new MatOfPoint2f();
			for (int i = 0; i < interestingReferenceBlobs.size(); ++i) {			
				interestingReferenceBlobs.get(i).convertTo(contour, CvType.CV_32FC2);
				Imgproc.approxPolyDP(contour, approx, 3, true);			
				if(approx.rows() == 3)
				{				
					//found a triangle ?
					Rect BB = Imgproc.boundingRect(interestingReferenceBlobs.get(i));
					if(BB.width > maxReferenceValue)
					{
						maxReferenceValue = BB.width;
						maxReferenceIdx = i;
						contour.copyTo(maxReferenceTarget);
					}
				}			
			}
			
			if (maxReferenceIdx >= 0 && maxReferenceIdx <= interestingReferenceBlobs.size())
			{
				//if we found the triangle, update the reference offset
				MatOfPoint tmpPt = new MatOfPoint();
				maxReferenceTarget.convertTo(tmpPt, CvType.CV_32S);
				referenceTarget.clear();
				referenceTarget.add(0, tmpPt);
				Rect BB = Imgproc.boundingRect(interestingReferenceBlobs.get(maxReferenceIdx));
				referenceTargetOffset = (BB.x + (BB.width/2)) - (Image.cols()/2) - defaultReferenceTargetOffset;
				//System.out.println("Reference: " + BB.width + "x" + BB.height + " " + (double)BB.width/BB.height );
			} 
			else
			{
				//Reference target not found
				referenceTarget.clear();
				referenceTargetOffset = 0;
			}
		} else {
			referenceTargetOffset = 0;
		}
		

		if(minIdx < 0 || minIdx > interestingBlobs.size())
		{
			return null;
		}
		
		//System.out.println("score: " + minValue);
		
		return interestingBlobs.get(minIdx);
	}
	

	private boolean AnalyzeTarget(Mat frame, boolean UpdateSaveImage)
	{
		long startTime = System.currentTimeMillis();
		boolean success = true;
		
		if(!openCVisGood) return false;
					
		// Constants (CALIBRATE ME!)
		//final int X0 = 174; //260; // inches (distance from target in sample image)
		//final int P0 = 44;  //32;    // pixels (height in pixels of sample image);
		
		Rect target = findTarget(frame);
		
		if (target == null)
		{
			bearing = BAD_BEARING;
			success = false; //:-(
		}
		else
		{
			int targetCenterX = target.x + (target.width / 2);
			int imageCenterX = (frame.cols() / 2) + referenceTargetOffset;
			int deltaX  = targetCenterX - imageCenterX;						

			//double deltaTheta = Math.atan(deltaX / ((X0 / (double) P0) * target.height));
			//bearing = deltaTheta * (180.0 / Math.PI);
			
			bearing = (61.0f/frame.cols())* (double)deltaX; //based on 61 degree field of view.
		}
		//System.out.println("bearing: " + bearing);		
		
		if(UpdateSaveImage) {
			//build image with target highlighted
			if(target != null)
			{
				Imgproc.rectangle(frame, target.tl(), target.br() , new Scalar(0, 0, 255, 255), 4);
			}
			//Add bearing text
			Imgproc.putText(frame, String.format("Bearing %.3f", bearing),  new Point(10,460), Core.FONT_HERSHEY_PLAIN, 2,  new Scalar(255,255,255,255));
			
			if(useReferenceTarget && !referenceTarget.isEmpty())
			{
				Imgproc.drawContours(frame, referenceTarget, 0, new Scalar(255, 0, 0, 255), 3);
				//add reference 
				Imgproc.putText(frame, String.format("R %d", referenceTargetOffset),  new Point(400,460), Core.FONT_HERSHEY_PLAIN, 2,  new Scalar(255,255,255,255));
			}

			synchronized(saveImageLock) {
				frame.copyTo(saveImage);
			}
		}						
		//System.out.println("AnalyzeTarget execution time: " + (System.currentTimeMillis() - startTime));
		
		return success;		
	}


	
	private class CaptureThread implements Runnable {
		
		private Mat lastFrame;
	
	    
		public CaptureThread()
		{
		    lastFrame = new Mat();
		    
			Thread thisThread = new Thread(this);
			//Lower the priority of this thread to try and prevent interfering with normal robot behavior 
			thisThread.setPriority(Thread.NORM_PRIORITY-2);
			thisThread.start();
			
		}
		
		public void run() {    
			if(!openCVisGood) return;
			
			long startTime;

			while (true) {
				startTime = System.nanoTime();

				captureImage();

				if(lastFrame != null && lastCapture != null){
					synchronized(lastCaptureLock){
						lastFrame.copyTo(lastCapture);
						lastCaptureTime = System.nanoTime();
					}
				}
				try {
					long runTime = (System.nanoTime() - startTime)/1000000;
					//System.out.println("Capture Thread: " + runTime);
					if(runTime < 66) {
						Thread.sleep(66 - runTime); //Capture at ~15 FPS  
					}
				} catch (InterruptedException e) {
					System.out.println("InterruptedException: " + e);
				}
			}
		}
		
		void captureImage()
		{
			if(!videoCapture.isOpened())
			{
				try {
					//use v4l2-ctl to turn off auto exposure
					//min=5 max=20000 step=1 default=156 value=5
					Runtime.getRuntime().exec(new String[]{"bash","-c","v4l2-ctl --set-ctrl=exposure_auto=1;v4l2-ctl --set-ctrl=exposure_absolute=10"});
				} catch (IOException e1) {
					e1.printStackTrace();
				}

				//attempt to open camera	
				if(!videoCapture.open(cameraIndex))
				{
					System.out.println("Error Opening Camera");
				}
			}
			
			//if it opened, attempt to process
			if(videoCapture.isOpened())
			{			
				synchronized(lastCaptureLock){
					videoCapture.read(lastFrame);	
				}
			}
		}
	}
	
	private class DSThread implements Runnable {
		
		public DSThread()
		{
			Thread thisThread = new Thread(this);
			//Lower the priority of this thread to try and prevent interfering with normal robot behavior 
			thisThread.setPriority(Thread.NORM_PRIORITY-2);
			thisThread.start();
		}
		
		public void run() {
			long startTime;
			
			while(true)
			{
				startTime = System.nanoTime();
				
				//create the latest image
				UpdateAnalysis(true);				
				
				if (saveImage != null && !saveImage.empty())
				{
					
					try {
						synchronized(saveImageLock) {
							MatOfInt my_params = new MatOfInt(Imgcodecs.CV_IMWRITE_PNG_COMPRESSION , 3);
							Imgcodecs.imwrite("/var/volatile/tmp/opencv-image.png", saveImage, my_params);
						}
					} catch (Exception e) {
						System.out.println("Exception saving image: " + e);
					}		
					
					try {
						NIVision.imaqReadFile(dashImage,"/var/volatile/tmp/opencv-image.png"); 
						CameraServer.getInstance().setImage(dashImage);
					} catch (VisionException e) {
						System.out.println("Error reading image from file: " + e);
					}
				}			
				try {
					long runTime = (System.nanoTime() - startTime)/1000000;
					//System.out.println("DS Thread: " + runTime);
					if(runTime < 1000) {
						Thread.sleep(1000 - runTime); //Process at 1Hz
					}
				} catch (InterruptedException e) {
					System.out.println("InterruptedException: " + e);
				}
			}
		}		
	}	
}
