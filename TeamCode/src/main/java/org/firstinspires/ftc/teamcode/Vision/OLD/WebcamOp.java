package org.firstinspires.ftc.teamcode.Vision.OLD;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

//@TeleOp
@Config
public class WebcamOp extends LinearOpMode {


    int cameraMonitorViewId = 0;
    WebcamName webcamName = null;
    OpenCvWebcam camera = null;
    SamplePipeline samplePipeline;

    double DetectWidth = 0;
    double DetectHeight = 0;

    @Override
    public void runOpMode(){
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class,"Webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        samplePipeline = new SamplePipeline();
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320 , 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                Log.e("Camera status", "Not opened");
            }
        });

        telemetry.addData("FPS", camera.getFps());
        telemetry.addData("Frames", camera.getFrameCount());
        telemetry.addLine();

        camera.setPipeline(samplePipeline);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Height", samplePipeline.getHeight());
            telemetry.addData("Width", samplePipeline.getWidth());
            telemetry.addData("X", samplePipeline.getX());
            telemetry.addData("Y", samplePipeline.getY());
            telemetry.addData("Area", samplePipeline.getArea());
            telemetry.update();
        }
    }

}