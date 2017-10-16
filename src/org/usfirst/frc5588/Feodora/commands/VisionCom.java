package org.usfirst.frc5588.Feodora.commands;

import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.usfirst.frc5588.Feodora.subsystems.Drive;

import edu.wpi.first.wpilibj.command.Subsystem;

public class VisionCom extends Subsystem{
	private Mat rgbThresholdOutput = new Mat();
	private Mat blurOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> convexHullsOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

	// Orange target, exposure 7:
	/*public float ThresholdRMin = 150;
    public float ThresholdRMax = 255;
    public float ThresholdGMin = 50;
    public float ThresholdGMax = 230;
    public float ThresholdBMin = 0;
    public float ThresholdBMax = 30;*/
	// Green target, exposure 7:
	public float ThresholdRMin = 0;
	public float ThresholdRMax = 10;
	public float ThresholdGMin = 100;
	public float ThresholdGMax = 180;
	public float ThresholdBMin = 10;
	public float ThresholdBMax = 90;

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

	/**
    * This is the primary method that runs the entire pipeline and updates the outputs.
    */
	@Override
	public void process(Mat source0) {
        // Step RGB_Threshold0:
        Mat rgbThresholdInput = source0;
        double[] rgbThresholdRed = {ThresholdRMin, ThresholdRMax};
        double[] rgbThresholdGreen = {ThresholdGMin, ThresholdGMax};
        double[] rgbThresholdBlue = {ThresholdBMin, ThresholdBMax};
        rgbThreshold(rgbThresholdInput, rgbThresholdRed, rgbThresholdGreen, rgbThresholdBlue, rgbThresholdOutput);

        // Step Blur0:
        Mat blurInput = rgbThresholdOutput;
        BlurType blurType = BlurType.MEDIAN;
        double blurRadius = 3.0;
        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step Find_Contours0:
        //Mat findContoursInput = blurOutput;
        Mat findContoursInput = rgbThresholdOutput;
        boolean findContoursExternalOnly = false;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

        // Step Convex_Hulls0:
        ArrayList<MatOfPoint> convexHullsContours = findContoursOutput;
        convexHulls(convexHullsContours, convexHullsOutput);

        // Step Filter_Contours0:
        ArrayList<MatOfPoint> filterContoursContours = convexHullsOutput;
        double filterContoursMinArea = 100.0;
        double filterContoursMinPerimeter = 0;
        double filterContoursMinWidth = 10;
        double filterContoursMaxWidth = 1000;
        double filterContoursMinHeight = 3;
        double filterContoursMaxHeight = 1000;
        double[] filterContoursSolidity = {0, 100};
        double filterContoursMaxVertices = 1000000;
        double filterContoursMinVertices = 5;
        double filterContoursMinRatio = 0;
        double filterContoursMaxRatio = 1000;
        filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

    }
	
	public Mat debugMat() {
        //return rgbThresholdOutput;
        return blurOutput;
        /*if (filterContoursOutput.size() > 0) {
            return filterContoursOutput.get(0);
        }
        return null;*/
    }

	/**
    * This method is a generated getter for the output of a RGB_Threshold.
    * @return Mat output from RGB_Threshold.
    */
	public Mat rgbThresholdOutput() {
        return rgbThresholdOutput;
    }

	/**
    * This method is a generated getter for the output of a Blur.
    * @return Mat output from Blur.
    */
	public Mat blurOutput() {
        return blurOutput;
    }

	/**
    * This method is a generated getter for the output of a Find_Contours.
    * @return ArrayList<MatOfPoint> output from Find_Contours.
    */
	public ArrayList<MatOfPoint> findContoursOutput() {
        return findContoursOutput;
    }

	/**
    * This method is a generated getter for the output of a Convex_Hulls.
    * @return ArrayList<MatOfPoint> output from Convex_Hulls.
    */
	public ArrayList<MatOfPoint> convexHullsOutput() {
		return convexHullsOutput;
    }

	/**
    * This method is a generated getter for the output of a Filter_Contours.
    * @return ArrayList<MatOfPoint> output from Filter_Contours.
    */
	public ArrayList<MatOfPoint> filterContoursOutput() {
		return filterContoursOutput;
    }


	/**
    * Segment an image based on color ranges.
    * @param input The image on which to perform the RGB threshold.
    * @param red The min and max red.
    * @param green The min and max green.
    * @param blue The min and max blue.
    * @param output The image in which to store the output.
    */
	private void rgbThreshold(Mat input, double[] red, double[] green, double[] blue, Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2RGB);
		Core.inRange(out, new Scalar(red[0], green[0], blue[0]), new Scalar(red[1], green[1], blue[1]), out);
    }

	/**
    * An indication of which type of filter to use for a blur.
    * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
    */
	enum BlurType{
		BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"), BILATERAL("Bilateral Filter");

		private final String label;

		BlurType(String label) {
			this.label = label;
        }

		public static BlurType get(String type) {
			if (BILATERAL.label.equals(type)) {
				return BILATERAL;
			}
			else if (GAUSSIAN.label.equals(type)) {
				return GAUSSIAN;
			}
			else if (MEDIAN.label.equals(type)) {
				return MEDIAN;
			}
			else {
				return BOX;
            }
        }

		@Override
		public String toString() {
			return this.label;
        }
	}

	/**
    * Softens an image using one of several filters.
    * @param input The image on which to perform the blur.
    * @param type The blurType to perform.
    * @param doubleRadius The radius for the blur.
    * @param output The image in which to store the output.
    */
    private void blur(Mat input, BlurType type, double doubleRadius,
        Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    /**
    * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
    * @param input The image on which to perform the Distance Transform.
    * @param type The Transform.
    * @param maskSize the size of the mask.
    * @param output The image in which to store the output.
    */
    private void findContours(Mat input, boolean externalOnly,
        List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

    /**
    * Compute the convex hulls of contours.
    * @param inputContours The contours on which to perform the operation.
    * @param outputContours The contours where the output will be stored.
    */
    private void convexHulls(List<MatOfPoint> inputContours,
        ArrayList<MatOfPoint> outputContours) {
        final MatOfInt hull = new MatOfInt();
        outputContours.clear();
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final MatOfPoint mopHull = new MatOfPoint();
            Imgproc.convexHull(contour, hull);
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int) hull.get(j, 0)[0];
                double[] point = new double[] {contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            outputContours.add(mopHull);
        }
    }


    /**
    * Filters out contours that do not meet certain criteria.
    * @param inputContours is the input list of contours
    * @param output is the the output list of contours
    * @param minArea is the minimum area of a contour that will be kept
    * @param minPerimeter is the minimum perimeter of a contour that will be kept
    * @param minWidth minimum width of a contour
    * @param maxWidth maximum width
    * @param minHeight minimum height
    * @param maxHeight maximimum height
    * @param Solidity the minimum and maximum solidity of a contour
    * @param minVertexCount minimum vertex Count of the contours
    * @param maxVertexCount maximum vertex Count
    * @param minRatio minimum ratio of width to height
    * @param maxRatio maximum ratio of width to height
    */
    private void filterContours(List<MatOfPoint> inputContours, double minArea,
        double minPerimeter, double minWidth, double maxWidth, double minHeight, double
        maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
        minRatio, double maxRatio, List<MatOfPoint> output) {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        //operation
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if (bb.width < minWidth || bb.width > maxWidth) continue;
            if (bb.height < minHeight || bb.height > maxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int)hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < solidity[0] || solid > solidity[1]) continue;
            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)    continue;
            final double ratio = bb.width / (double)bb.height;
            if (ratio < minRatio || ratio > maxRatio) continue;
			output.add(contour);
        }
    }

public ArrayList<MatOfPoint> finalContours() {
    return filterContoursOutput();
}

@Override
protected void initDefaultCommand() {
	// TODO Auto-generated method stub
}
private static VisionCom instance = new VisionCom();
	public static VisionCom getInstance()
    {
    	return instance;
    }
    
}



