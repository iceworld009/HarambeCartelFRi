package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;

import java.util.concurrent.ForkJoinPool;

public class Servos {
    private HardwareClass hardwareClass;
    private CRServo rise1, rise2;
    private static Servos single_instance = null;
    private double medianError = -1;

    public Servos(HardwareClass hardwareClass, Telemetry telemetry , HardwareMap hardwareMap) {
        this.hardwareClass = hardwareClass;
        this.rise1 = hardwareClass.rise1;
        this.rise2 = hardwareClass.rise2;
    }

    public void RiseUp(double power){
        rise1.setPower(power);
        rise2.setPower(power);
    }

    public void sleep(int sec){
        try {
            Thread.sleep(sec);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public static synchronized Servos getInstance(HardwareMap hardwareMap , Telemetry telemetry ){
        if(single_instance == null){
            single_instance = new Servos(HardwareClass.getInstance(hardwareMap), telemetry , hardwareMap);
        }
        return single_instance;
    }
}
