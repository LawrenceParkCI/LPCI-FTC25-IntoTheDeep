package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.components.camera.Camera;
import org.firstinspires.ftc.teamcode.components.camera.pipelines.SampleDetector;

@TeleOp(name = "Camera Test", group = "Test")
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Camera camera = new Camera(640, 480, "webcam", hardwareMap);
        SampleDetector pipeline = new SampleDetector();
        pipeline.setMorphIterations(2);
        pipeline.drawContours(true);

        camera.open();
        camera.setPipeline(pipeline);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Gamepad currGamePad = new Gamepad();
        Gamepad prevGamePad = new Gamepad();

        waitForStart();
        while(opModeIsActive()){
            prevGamePad.copy(currGamePad);
            currGamePad.copy(gamepad1);

            if(currGamePad.a && !prevGamePad.a){
                pipeline.showBinary(true);
            }

            if(currGamePad.b && !prevGamePad.b){
                pipeline.showBinary(false);
            }

            telemetry.addData("Status", "Active");
            telemetry.addData("Detection: ", pipeline.getImgPoints().dump());
            telemetry.update();
        }
    }
}
