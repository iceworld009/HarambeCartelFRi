package org.firstinspires.ftc.teamcode.SubSys;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;



public class Limelight {
    private static Limelight single_instance = null;
    private boolean running = false;
    public int pipeline = -1;
    private Limelight3A limelight;
    double Tx, Ty;
    public Limelight(HardwareClass hardwareClass, Telemetry telemetry , HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
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

    public void setPipeline(int pipe){
        limelight.pipelineSwitch(pipe);
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