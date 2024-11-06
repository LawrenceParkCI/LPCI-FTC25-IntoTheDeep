package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.components.camera.Camera;
import org.firstinspires.ftc.teamcode.components.camera.util.MatLoader;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Intrinsics Calibration OpMode, Images should be taken a multiple angles
 * @author Connor Feeney
 */
@TeleOp(name = "Intrinsics Calibration", group = "Calibration")
public class IntrinsicsCalibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Create and setup our camera to grab and use frames
        Camera camera = new Camera(640, 480, "webcam", hardwareMap);
        camera.open();
        Pipeline pipeline = new Pipeline();
        camera.setPipeline(pipeline);

        //Gamepad objects to make rising edge detector
        Gamepad currGamePad = new Gamepad();
        Gamepad prevGamePad = new Gamepad();

        Mat cameraMatrix = Mat.eye(3,3, CvType.CV_64F); //3x3 identity matrix for the camera intrinsics
        Mat distCoeffs = new Mat(); //Basic matrix for distortion coefficients
        List<Mat> objPoints = new ArrayList<>(); //Our object points
        List<Mat> imgPoints = new ArrayList<>(); //Our image points

        //Telemetry Check booleans
        boolean cameraMatWrite;
        boolean distCoeffsMatWrite;
        boolean calibrated = false;

        int imgCount = 0;

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            //Setup for rising edge detector
            prevGamePad.copy(currGamePad);
            currGamePad.copy(gamepad1);

            //Process image points and object point on A button press
            if(currGamePad.a && !prevGamePad.a){
                pipeline.procImg(objPoints, imgPoints);
                imgCount++;
            }

            //Calibrate camera on B button press
            if(currGamePad.b && !prevGamePad.b){
                calibrateCamera(cameraMatrix, distCoeffs, objPoints, imgPoints);

                MatLoader loader = new MatLoader(hardwareMap);
                cameraMatWrite = loader.writeMatFile(cameraMatrix, "camMat.mat");
                distCoeffsMatWrite = loader.writeMatFile(distCoeffs, "distCoeffs.mat");
                imgCount = 0;

                if(cameraMatWrite && distCoeffsMatWrite){
                    calibrated = true;
                }
            }

            //Do all the telemetry junk
            telemetry.addData("Status: ", "Active");
            telemetry.addData("Image Count: ", imgCount);
            if(calibrated){
                telemetry.addData("Camera Matrix: ", cameraMatrix.dump());
                telemetry.addData("Distortion Coefficients: ", distCoeffs.dump());
            }
            telemetry.update();
        }
    }

    /**
     * Calibrate your camera and get you intrinsics matrices
     * @param cameraMatrix The 3x3 identity matrix for your new camera matrix
     * @param distCoeffs The matrix for your distortion coefficients
     * @param objPoints Your object points
     * @param imgPoints Your image points
     */
    private void calibrateCamera(Mat cameraMatrix, Mat distCoeffs, List<Mat> objPoints, List<Mat> imgPoints){
        telemetry.addLine("Calibrating...");
        telemetry.update();
        List<Mat> rvecs = new ArrayList<>(); //Ignore
        List<Mat> tvecs = new ArrayList<>(); //Ignore

        //Calibrate the camera using built in opencv function
        if(!objPoints.isEmpty() && !imgPoints.isEmpty()){
            Calib3d.calibrateCamera(objPoints, imgPoints,new Size(19, 19), cameraMatrix, distCoeffs, rvecs, tvecs);
            objPoints.clear();
            imgPoints.clear();
        }
    }

    /**
     * Simple pipeline for getting calibration points for camera intrinsics
     * @author Connor Feeney
     */
    private static class Pipeline extends OpenCvPipeline {
        Mat frame = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(frame);
            return input;
        }

        /**
         * Processes an image for image points and object points from a chessboard pattern for camera calibration
         * @param objPoints The list for object points to be stored in
         * @param imgPoints The list for image points to be stored in
         */
        public void procImg(List<Mat> objPoints, List<Mat> imgPoints){
            Size board = new Size(19,19); //Function uses 20x20 chessboard
            //Populate a simple object point list
            MatOfPoint3f objp = new MatOfPoint3f();
            List<Point3> objpList = new ArrayList<>();
            for(int i = 0; i < board.height; i++){
                for(int j = 0; j < board.width; j++){
                    objpList.add(new Point3(j, i, 0));
                }
            }
            objp.fromList(objpList);

            Mat cImg = frame.clone(); //Clone the input image data for processing

            //Process image
            Imgproc.cvtColor(cImg, cImg, Imgproc.COLOR_RGB2GRAY);
            MatOfPoint2f corners = new MatOfPoint2f();
            boolean found = Calib3d.findChessboardCorners(cImg, board, corners);
            if(found){
                objPoints.add(objp);
                imgPoints.add(corners);
            }
            cImg.release(); //Release the cloned image
        }
    }
}
