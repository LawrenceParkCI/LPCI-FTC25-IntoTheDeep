package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.MatLoader;

import org.opencv.core.Mat;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * A wrapper class around the {@link OpenCvCamera} class
 * @author Connor Feeney
 */
public class Camera {
    HardwareMap hardwareMap; //The hardware map for whatever OpMode is currently running, needed for app context
    private final OpenCvCamera camera; //The actual camera object

    private int width = 0; //Width of the camera frame in pixels
    private int height = 0; //Height of the camera frame in pixels

    private Mat camMat = new Mat(); //The camera intrinsics matrix, must be calibrated for different resolutions
    private Mat distCoeffs = new Mat(); //The distortion coefficients of the camera

    /**
     * @param width Width of the camera frame in pixels
     * @param height Height of the camera frame in pixels
     * @param webcam The name of the webcam in your config
     * @param hardwareMap The OpModes hardwareMap
     */
    public Camera(int width, int height, String webcam, HardwareMap hardwareMap){
        //Get camera monitor
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //Create Camera
        WebcamName webcamName = hardwareMap.get(WebcamName.class, webcam);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        //Store params for later use
        this.width = width;
        this.height = height;
        this.hardwareMap = hardwareMap;
    }

    /**
     * Load the camera intrinsic files
     * @return True if the files loaded false if the files failed to load
     */
    public boolean loadIntrinsics(){
        //Release old mats to prevent memory leak
        if(camMat != null){
            camMat.release();
        }

        if(distCoeffs != null){
            distCoeffs.release();
        }

        MatLoader loader = new MatLoader(hardwareMap);
        camMat = loader.loadMatFile("camMat.mat");
        distCoeffs = loader.loadMatFile("distCoeffs.mat");
        return camMat != null && distCoeffs != null;
    }

    /**
     * Open the camera
     */
    public void open(){
        //Open the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //Start the camera streaming
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            //I don't really know what i would do if an error happens so im just gunna let the program crash
            @Override
            public void onError(int errorCode) {/*Pass*/}
        });
    }

    /**
     * Pause view port to save cpu time
     */
    public void pauseViewPort(){
        camera.pauseViewport();
    }

    /**
     * Resume view port
     */
    public void resumeViewPort(){
        camera.resumeViewport();
    }

    /**
     * Set the camera pipeline
     * @param pipeline desired pipeline to use
     */
    public void setPipeline(OpenCvPipeline pipeline){
        camera.setPipeline(pipeline);
    }

    /**
     * Returns the base {@link OpenCvCamera}
     * @return {@link OpenCvCamera}
     */
    public OpenCvCamera getCamera(){
        return camera;
    }

    /**
     * Returns the calibrated camera matrix, can be calibrated with {@link org.firstinspires.ftc.teamcode.calibration.IntrinsicsCalibration}
     * @return camera matrix
     */
    public Mat getCamMat(){
        return camMat;
    }

    /**
     * Returns the calibrated distortion coefficients, can be calibrated with {@link org.firstinspires.ftc.teamcode.calibration.IntrinsicsCalibration}
     * @return distortion coefficients
     */
    public Mat getDistCoeffs(){
        return distCoeffs;
    }
}
