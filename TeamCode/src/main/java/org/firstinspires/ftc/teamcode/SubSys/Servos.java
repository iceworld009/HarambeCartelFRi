package org.firstinspires.ftc.teamcode.SubSys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;
public class Servos {
    private HardwareClass hardwareClass;
    private static Servos single_instance = null;
    Servo hood = hardwareClass.angle;

    public Servos(HardwareClass hardwareClass, Telemetry telemetry , HardwareMap hardwareMap) {
        this.hardwareClass = hardwareClass;
    }

    public void hoodSetPos(double pos){ //0 = down, 1 = up
        hood.setPosition(HardwareClass.hoodDown + (HardwareClass.hoodUp - HardwareClass.hoodDown) * pos);
    }

    public static synchronized Servos getInstance(HardwareMap hardwareMap , Telemetry telemetry ){
        if(single_instance == null){
            single_instance = new Servos(HardwareClass.getInstance(hardwareMap), telemetry , hardwareMap);
        }
        return single_instance;
    }
}