package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.TurretPID;

public class Turret {

    //Declarations
    DcMotorEx TurretMotor;
    //Gamepad gm;

    Telemetry telemetry = null;

    public static double kp = 0.006, ki = 0, kd = 0.00002; //prima data bai p si apoi pula ;)
    public static double target = 100;
    TurretPID generalPID = null;
    HardwareClass hardwareClass = null;

    //Singleton
    private static Turret single_instance = null;
    Thread thread = null;
    private boolean running = false;

    public Turret(HardwareClass hardwareClass, Telemetry telemetry){
        this.TurretMotor = hardwareClass.turret;
        this.telemetry = telemetry;
        this.hardwareClass = hardwareClass;
    }

    public void setup(){
        running = true;
        TurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        generalPID = TurretPID.getInstance(TurretMotor, "TR", "pid", telemetry);
        generalPID.setCoefficients(kp, ki, kd);
        generalPID.start();
    }


    public void wait(int sec) throws InterruptedException{
        generalPID.delay(sec);
    }

    public int getPosition(){
        return TurretMotor.getCurrentPosition();
    }


    public void resetMotor(){
        TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void activate(){
        generalPID.activetePower();
    }

    public void still(){

    }

    public void goToPosition(double target){
        generalPID.setReference(target);
    }
    public void setPID(double p , double i, double d){
        generalPID.setCoefficients(p,i,d);
    }

    public void stop(){
        running = false;
        if (generalPID != null) {
            generalPID.stop();
        }
        TurretMotor.setPower(0);
    }


    public void resetPoition(){
        generalPID.stop();
        TurretMotor.setPower(0.2);
        sleep(500);
        resetMotor();
        TurretMotor.setTargetPosition(305);
        resetMotor();
        generalPID.start();
    }

    public void resume(){
        running = true;
        generalPID.start();
    }

    public void kill(){

    }

    public void powerOFF(){
        generalPID.deactivatePower();
    }

    public void powerOn(){
        generalPID.activetePower();
    }

    public void sleep(int sec){
        try {
            Thread.sleep(sec);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public boolean getStatus(){
        return running;
    }

    public static synchronized Turret getInstance(HardwareMap hardwareMap, Telemetry telemetry){
        if(single_instance == null){
            single_instance = new Turret(HardwareClass.getInstance(hardwareMap), telemetry);
        }
        return single_instance;
    }
}
