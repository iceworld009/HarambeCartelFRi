package org.firstinspires.ftc.teamcode.Threads;

import android.health.connect.datatypes.BasalBodyTemperatureRecord;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Motors;

import java.util.List;
import java.util.Objects;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Limelight {
    Thread thread = null;
    private static Limelight single_instance = null;
    private boolean running = false;
    public double distance;
    public int pipeline = -1;

    public int possiblePipeline = -1;

    double maxX = -100;
    private Limelight3A limelight;
    double Tx, Ty;
    public Limelight(HardwareClass hardwareClass, Telemetry telemetry , HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void start() {  // citit in auto
        running = true;
        if (thread == null || !thread.isAlive()) {
            thread = new Thread(() -> {
                int tryPipe = 1;
                while (running) {
                    setPipeline(tryPipe);

                    try { Thread.sleep(200); } catch (InterruptedException e) { break; }

                    LLResult result = limelight.getLatestResult();
                    if (result != null && result.isValid()) {
                        this.pipeline = tryPipe;
                        this.running = false;
                        break;
                    }

                    tryPipe = (tryPipe >= 3) ? 1 : tryPipe + 1;
                }
            });
            thread.start();
        }
    }

    public void startPreSeek() { //citire inainte de auto
        running = true;
        if (thread == null || !thread.isAlive()) {
            thread = new Thread(() -> {
                int tryPipe = 1;
                while (running) {
                    setPipeline(tryPipe);

                    try { Thread.sleep(200); } catch (InterruptedException e) { break; }

                    LLResult result = limelight.getLatestResult();
                    if (result != null && result.isValid() && (result.getTx()<maxX || maxX ==-100)) {
                        this.possiblePipeline = tryPipe;
                        maxX = result.getTx();
                    }

                    tryPipe = (tryPipe >= 3) ? 1 : tryPipe + 1;
                }
            });
            thread.start();
        }
    }

    public double getXPos(){
        LLResult result = limelight.getLatestResult();
        Tx = result.getTx();
        return Tx;
    }

    public boolean checkResults(){
        LLResult result = limelight.getLatestResult();
        return result.isValid();
    }
    public double getYPos() {
        LLResult result = limelight.getLatestResult();
        Ty = result.getTy();
        return Ty;
    }

    public double getArea(){
        LLResult result = limelight.getLatestResult();
        return result.getTa();
    }

    public int checkApriltagResults() {
        return this.pipeline;
    }

    public double getDistanceOD(double x, double y, int color){ //Formula automata pentru cele doua cosuri
        if(color == 0){//red
            distance = Math.sqrt(Math.pow(HardwareClass.redX-x,2)+Math.pow(HardwareClass.redY-y,2));
        }
        else
            distance = Math.sqrt(Math.pow(HardwareClass.blueX-x,2)+Math.pow(HardwareClass.blueY-y,2));

        return distance * 2.54;
    }

    public double getDistanceODMan(double x, double y, double targetX, double targetY){
        return Math.sqrt(Math.pow(targetX-x,2)+Math.pow(targetY-y,2)) * 2.54;
    }

    public double getRobotAngle(double x, double y, int target){
        if(target == 0)
            return Math.atan2(Math.abs(HardwareClass.redY-y),Math.abs(HardwareClass.redX-x));
        return Math.atan2(Math.abs(HardwareClass.blueY-y),Math.abs(HardwareClass.blueY-x));
    }

    public double getRPM(double d) {
        double r = 0.012 * d * d - 1.9 * d + 2070;
        r = Math.max(Math.min(r,3000),100);
        return r;
    }
    public double getHood(double d) {
        if (d < 130)
            return (d - 100) * (9.0 / 30.0);
        else if (d < 170)
            return 9 - (d - 130) * (1.0 / 40.0);
        else
            return 6 + (d - 170) * (5.0 / 150.0);
    }


    public void setPipeline(int pipe){
        limelight.pipelineSwitch(pipe);
    }

    public void wait(int sec){
        try {
            Thread.sleep(sec);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void stop(){
        running = false;
    }
    public void setup(){
        running = true;
    }
    public boolean getStatus() {
        return running;
    }
    public static synchronized Limelight getInstance(HardwareMap hardwareMap , Telemetry telemetry ){
        if(single_instance == null){
            single_instance = new Limelight(HardwareClass.getInstance(hardwareMap), telemetry, hardwareMap);
        }
        return single_instance;
    }
}
