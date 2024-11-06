package org.firstinspires.ftc.teamcode.components.camera.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * FTC Into The Deep 24-25 <br>
 * An {@link OpenCvPipeline} for detecting samples
 * <br><br>
 * Last Updated: November 6th, 2024
 * @author Connor Feeney
 */
public class SampleDetector extends OpenCvPipeline {
    //Binary Image fields
    private boolean showBinary = false;
    private int morphIterations = 1;
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    private final Mat binary = new Mat();

    //Draw Image fields
    private boolean drawContours = false;
    private final Mat drawMat = new Mat();

    //Upper lower masks
    Scalar lowerYellow = new Scalar(20, 80, 50);
    Scalar upperYellow = new Scalar(40, 255, 255);

    //Image points results
    private List<MatOfPoint> contours = new ArrayList<>();
    private final MatOfPoint2f imgPoints = new MatOfPoint2f();

    @Override
    public Mat processFrame(Mat input) {
        //Temporarily store detection stuff
        List<MatOfPoint> ret = new ArrayList<>();

        //Create a binary image mask
        input.copyTo(binary);
        mask(binary, lowerYellow, upperYellow);

        //Find contours in the binary image
        Mat hierarchy = new Mat();
        Imgproc.findContours(binary, ret, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();

        double largestContour = 0;
        MatOfPoint2f tempDetection = new MatOfPoint2f();
        for(int i = 0; i < ret.size(); i++){
            //Approximate Curve
            MatOfPoint2f contour2f = new MatOfPoint2f(ret.get(i).toArray());
            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, contour2f, epsilon, true);

            //Store approximation
            ret.get(i).release();
            ret.set(i, new MatOfPoint(contour2f.toArray()));

            //Store the current largest contour
            double contourArea = Imgproc.contourArea(ret.get(i));
            if(contourArea > largestContour){
                largestContour = contourArea;
                contour2f.copyTo(tempDetection);
            }

            //Release float to prevent memory leak
            contour2f.release();
        }

        //Permanently move the temp detection to the imgPoints
        tempDetection.copyTo(imgPoints);
        tempDetection.release();

        if(drawContours){
            input.copyTo(drawMat);
            //Draw contour lines
            Imgproc.drawContours(drawMat, ret, -1, new Scalar(0, 255, 0), 3);

            //Draw Corner Points
            Point[] points = imgPoints.toArray();
            for(Point point : points){
                Imgproc.circle(drawMat, new Point(point.x, point.y), 5, new Scalar(255, 0, 0));
            }
        }

        //Swap pointers and clear old mats
        List<MatOfPoint> oImgPoints = contours;
        contours = ret;
        for(MatOfPoint p : oImgPoints){
            p.release();
        }
        oImgPoints.clear();

        //Return View Port Mat
        if(drawContours && !showBinary){
            return drawMat;
        } else if (showBinary) {
            return binary;
        }
        return input;
    }

    /**
     * Perform and HSV color mask on an image
     * @param src The image to mask
     * @param lowerMask The lower HSV values to mask to
     * @param upperMask The upper HSV values to mask to
     */
    private void mask(Mat src, Scalar lowerMask, Scalar upperMask){
        //Convert src to HSV color space
        Imgproc.cvtColor(src, src, Imgproc.COLOR_RGB2HSV);

        //Mask the image
        Core.inRange(src, lowerMask, upperMask, src);

        //Perform Dilation and Erosion morphs
        for(int i = 0; i < morphIterations; i++){
            Imgproc.morphologyEx(src, src, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(src, src, Imgproc.MORPH_CLOSE, kernel);
        }
    }

    /**
     * Get the current contours list
     * @return List of current contours
     */
    public List<MatOfPoint> getContours(){
        return contours;
    }

    /**
     * Get the current detected sample
     * @return Current detected sample
     */
    public MatOfPoint2f getImgPoints(){
        return imgPoints;
    }

    /**
     * Should the binary image be shown in the view port
     * @param showBinary True to show Binary
     */
    public void showBinary(boolean showBinary){
        this.showBinary = showBinary;
    }

    /**
     * Should contours be drawn
     * @param drawContours True to draw Contours
     */
    public void drawContours(boolean drawContours){
        this.drawContours = drawContours;
    }

    /**
     * Set how much filtering an image should go through
     * @param morphIterations The number of filtering iterations
     */
    public void setMorphIterations(int morphIterations){
        this.morphIterations = morphIterations > 0 ? morphIterations : 1;
    }
}
