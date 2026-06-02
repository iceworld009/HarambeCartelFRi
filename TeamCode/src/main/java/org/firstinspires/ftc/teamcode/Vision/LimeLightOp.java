package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Point;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.opencv.core.Mat;

import java.util.List;

@TeleOp
@Config
public class LimeLightOp extends LinearOpMode {

    private Limelight3A limelight;

    public static int field = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                if (result.isValid()) {
                    double posX = convertToNewRange(result.getTx() , -16 , 16 , -3 , 3);
                    double posY = convertToNewRange(result.getTy() , -10 , 10 , 600 , 1200);
                    double height = result.getColorResults().size();
                    telemetry.addData("tx", posX);
                    telemetry.addData("height", height);
                    telemetry.addData("ty", posY);
                    telemetry.addData("tync", result.getTxNC());

                    telemetry.addData("Botpose", botpose.toString());

                    // Access barcode results
                    List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                    for (LLResultTypes.BarcodeResult br : barcodeResults) {
                        telemetry.addData("Barcode", "Data: %s", br.getData());
                    }

                    // Access classifier results
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }

                    // Access detector results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }

                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }

                    // Access color results
                    angle(result);
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
        limelight.stop();
    }

    public void angle(LLResult result){
        double width = 0 , height = 0;
        double Mx = -999 , mx = 999, My = -999, my = 999;
        double i = 0, Iy = 0;

        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
        for (LLResultTypes.ColorResult cr : colorResults) {
            telemetry.addData("corners: ",  cr.getTargetCorners());
            for(List<Double> point : cr.getTargetCorners()){
                double x = point.get(0);
                double y = point.get(1);
                telemetry.addLine("Corner: " + "X:" + x +"; Y:" + y + "; Index: " + i);

                if(Mx < x){ Mx = x;}
                if(My < y){ My = y; Iy = i;}
                if(mx > x) mx = x;
                if(my > y) my = y;
                i++;
            }
        }

        width = Math.abs(Mx - mx);
        height = Math.abs(My - my);

        if(height > width - field){
            telemetry.addLine("VERTICAL " + height + " " + width);
        }
        else{
            telemetry.addLine("HORIZONTAL" + height + " " + width);
        }
    }


    public double convertToNewRange(double value, double oldMin, double oldMax, double newMin, double newMax){
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }


}