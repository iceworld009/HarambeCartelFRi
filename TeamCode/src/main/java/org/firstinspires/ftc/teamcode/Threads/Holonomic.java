package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;

public class Holonomic {

    //Declarations
    DcMotor FR , FL , BL , BR;
    Gamepad gm, gm2;

    float decl = 1;

    private double BLPower , BRPower , FRPower , FLPower, totalPower = 0;
    private double power , turn, max , x , y;
    private double sin , cos , theta;

    //Singleton
    private static Holonomic single_instance = null;
    Thread thread = null;
    private boolean running = false;

    public Holonomic(HardwareClass hardwareClass, Gamepad gamepad1, Gamepad gamepad2){
        this.FR = hardwareClass.FR;
        this.FL = hardwareClass.FL;
        this.BR = hardwareClass.BR;
        this.BL = hardwareClass.BL;

        this.gm = gamepad1;
        this.gm2 = gamepad2;
    }

    public void start(){
        running = true;
        if(thread == null || !thread.isAlive()){
            thread = new Thread(() ->{
                while(true){
                    if(running){
                        x = gm.left_stick_x;
                        y = -(gm.left_stick_y);
                        turn = (gm.right_stick_x ) * decl;

                        theta = Math.atan2(y , x);
                        power = Math.hypot(x, y);

                        sin = Math.sin(theta - Math.PI/4);
                        cos = Math.cos(theta - Math.PI/4);
                        max = Math.max(Math.abs(sin) , Math.abs(cos));

                        FLPower = power * cos/max + turn;
                        FRPower = power * sin/max - turn;
                        BLPower = power * sin/max + turn;
                        BRPower = power * cos/max - turn;

                        FL.setPower(FLPower + totalPower + 0.09);
                        FR.setPower(FRPower + totalPower + 0.09);
                        BL.setPower(BLPower + totalPower - 0.09);
                        BR.setPower(BRPower + totalPower - 0.09);
                    }
                }
            });
        }
        thread.start();
    }

    public void accel(){
        decl = 1;
    }

    public void decel(){
        decl = 0.5f;
    }

    public void setup(){
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
    }

    public void stop(){
        running = false;
    }


    public void restart(){
        running = true;
    }

    public boolean getStatus(){
        return running;
    }

    public static synchronized Holonomic getInstance(HardwareMap hardwareMap, Gamepad gm, Gamepad gm2){
        if(single_instance == null){
            single_instance = new Holonomic(HardwareClass.getInstance(hardwareMap), gm, gm2);
        }
        return single_instance;
    }
}