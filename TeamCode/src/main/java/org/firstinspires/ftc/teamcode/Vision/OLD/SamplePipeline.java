package org.firstinspires.ftc.teamcode.Vision.OLD;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class SamplePipeline extends OpenCvPipeline {

    //Low
    public static double L1 = 7;
    public static double L2 = 120;
    public static double L3 = 90;

    public static double H1 = 70;
    public static double H2 = 255;
    public static double H3 = 255;

    /**
     *  Detection 4 Yellow
     *
    public static double L1 = 7;
    public static double L2 = 110;
    public static double L3 = 180;

    public static double H1 = 45;
    public static double H2 = 255;
    public static double H3 = 255;

     */

    /**
     *  Detection 4 Red
     *
     public static double L1 = 0;
     public static double L2 = 100;
     public static double L3 = 60;

     public static double H1 = 10;
     public static double H2 = 255;
     public static double H3 = 255;

     */

    /**
     *  Detection 4 Blue
     *
     public static double L1 = 70;
     public static double L2 = 180;
     public static double L3 = 120;

     public static double H1 = 150;
     public static double H2 = 230;
     public static double H3 = 190;

     */

    public static int roiX = 120; // Starting x-coordinate
    public static int roiY = 100; // Starting y-coordinate
    public static int roiWidth = 100; // Width of the ROI
    public static int roiHeight = 100; // Height of the ROI

    private double area;

    private double x , y;
    private double height , width;

    Rect roi = null;

    int running = 0;

    @Override
    public Mat processFrame(Mat input){
        // Create the ROI rectangle

        Mat hsvFrame = new Mat();
        Mat thresholdFrame = new Mat();
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);// convert colors from RGB to HSV


        //Converting the image to binary ( differentiate colors by a certain threshold )
        Scalar lowerThreshold = new Scalar(L1 , L2 , L3);
        Scalar upperThreshold = new Scalar(H1 , H2 , H3);

        // take every pixel and see if it is in the given threshold
        Core.inRange(hsvFrame,lowerThreshold,upperThreshold,thresholdFrame);

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        Imgproc.findContours(thresholdFrame, contours,new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect lowBox = null;
        double lowest = 0;

        for(MatOfPoint contoru : contours){
            Rect boundBox = Imgproc.boundingRect(contoru);
            if(boundBox.y > lowest && boundBox.area() > 4500 && boundBox.area() < 15000){
                lowBox = boundBox;
                lowest = boundBox.y;
                height = lowBox.height;
                width = lowBox.width;
                x = lowBox.x;
                y = lowBox.y;
                area = lowBox.area();
            }
        }

        if (lowBox != null) {
            // Adjust bounding box coordinates to match the original imag
            running = 0;
            // Draw the bounding box on the original image
            Imgproc.rectangle(input, lowBox, new Scalar(255, 0, 0));

            // Draw an arrowed line to the detected object
            Imgproc.arrowedLine(input, new Point(0, 0), new Point(x + width/2, y + height/2), new Scalar(0, 255, 0));
        }

        // Release memory
        hsvFrame.release();
        thresholdFrame.release();
        running--;

        return input;
    }

    public void setBounds(int x , int y){
        roiWidth = x;
        roiHeight = y;
    }

    public void detect(){
        running = 10;
    }

    public void defaultBounds(){
        roiWidth = 100;
        roiHeight = 100;
    }

    public void setRed(){
        L1 = 0;
        L2 = 100;
        L3 = 60;

        H1 = 10;
        H2 = 255;
        H3 = 255;
    }

    public void setYellow(){
        L1 = 1;
        L2 = 150;
        L3 = 100;

        H1 = 45;
        H2 = 255;
        H3 = 255;
    }

    public void setBlue(){
        L1 = 70;
        L2 = 180;
        L3 = 120;

        H1 = 150;
        H2 = 230;
        H3 = 190;
    }

    public double convertToNewRange(double value, double oldMin, double oldMax, double newMin, double newMax){
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }

    public double getHeight(){
        return height;
    }
    public double getArea(){
        return area;
    }

    public double getWidth(){
        return width;
    }

    public double getX(){
        return x - 160 + width/2;
    }

    public double getY(){
        return -y + 120 + height/2;
    }

}
