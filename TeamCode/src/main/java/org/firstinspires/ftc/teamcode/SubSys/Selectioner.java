package org.firstinspires.ftc.teamcode.SubSys;

import static org.firstinspires.ftc.teamcode.Utils.GeneralUtils.*;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Utils.Color;

public class Selectioner{
    private static Selectioner selectionerInstance = null;
    private final Servo selectTop, selectBotR, selectBotL;
    private final RevColorSensorV3 colorTop, colorBotR, colorBotL,colorTop2, colorBotR2, colorBotL2;
    public volatile Color resultTop;
    public volatile Color resultBotR;
    public volatile Color resultBotL;
    public boolean ballsfull = false;
    private Telemetry telemetry;
    int greenPos = -1;

    public Selectioner(HardwareClass hardwareClass, Telemetry telemetry) {
        selectTop = hardwareClass.selectTop;   // 1
        selectBotL = hardwareClass.selectBotL; // 2
        selectBotR = hardwareClass.selectBotR; // 3

        colorTop = hardwareClass.colorTop;       // 1
        colorBotL = hardwareClass.colorBotL;     // 2
        colorBotR = hardwareClass.colorBotR;     // 3

        colorTop2 = hardwareClass.colorTop2;     // 1
        colorBotR2 = hardwareClass.colorBotR2;   // 2
        colorBotL2 = hardwareClass.colorBotL2;   // 3

        this.telemetry = telemetry;
    }


    public void unloadBalls(){  //Deprecated
        rightServoUp();
        sleep(HardwareClass.bratBetween);
        leftServoUp();
        sleep(HardwareClass.bratBetween);
        topServoUp();
        sleep(HardwareClass.bratBetween);
    }

    public void unloadBallsSlow(){  //Deprecated
        rightServoUp();
        sleep(HardwareClass.bratBetween+45);
        leftServoUp();
        sleep(HardwareClass.bratBetween+45);
        topServoUp();
        sleep(HardwareClass.bratBetween+45);
    }

    public void setTagPos(int pos){
        greenPos = pos;
    }

    private void topServoUp() {
        selectTop.setPosition(HardwareClass.selectTopHIGH);
        sleep(HardwareClass.bratDelay );
        selectTop.setPosition(HardwareClass.selectTopLOW);
    }

    private void leftServoUp(){
        selectBotL.setPosition(HardwareClass.selectBotLHIGH);
        sleep(HardwareClass.bratDelay );
        selectBotL.setPosition(HardwareClass.selectBotLLOW);
    }

    private void rightServoUp(){
        selectBotR.setPosition(HardwareClass.selectBotRHIGH);
        sleep(HardwareClass.bratDelay);
        selectBotR.setPosition(HardwareClass.selectBotRLOW);

    }

    public void resetServos(){
        selectTop.setPosition(HardwareClass.selectTopLOW);
        selectBotL.setPosition(HardwareClass.selectBotLLOW);
        selectBotR.setPosition(HardwareClass.selectBotRLOW);
    }

    private int getPurplePos() {
        if(resultTop == Color.PURPLE) return 1;
        else if(resultBotL == Color.PURPLE) return 2;
        else if(resultBotR == Color.PURPLE) return 3;
        return -1;
    }

    private int getGreenPos() {
        if(resultTop == Color.GREEN) return 1;
        else if(resultBotL == Color.GREEN) return 2;
        else if(resultBotR == Color.GREEN) return 3;
        return -1;
    }

    public void shootPurple() {
        if(resultTop == Color.PURPLE) {
            resultTop = Color.INVALID;
            topServoUp();
        } else if(resultBotL == Color.PURPLE) {
            resultBotL = Color.INVALID;
            leftServoUp();
        } else if(resultBotR == Color.PURPLE) {
            resultBotR = Color.INVALID;
            rightServoUp();
        }
    }
    public void shootGreen() {
        if(resultTop == Color.GREEN) {
            resultTop = Color.INVALID;
            topServoUp();
        } else if(resultBotL == Color.GREEN) {
            resultBotL = Color.INVALID;
            leftServoUp();
        } else if(resultBotR == Color.GREEN) {
            resultBotR = Color.INVALID;
            rightServoUp();
        }
    }

    public void telemetry() {
        telemetry.addData("Top Color", resultTop);
        telemetry.addData("Right Color", resultBotR);
        telemetry.addData("Left Color:", resultBotL);
        telemetry.addData("Top R", colorTop.red());
        telemetry.addData("Top G", colorTop.green());
        telemetry.addData("Top B", colorTop.blue());
        telemetry.addData("Top A", colorTop.alpha());
        telemetry.addData("Left R", colorBotL.red());
        telemetry.addData("Left G", colorBotL.green());
        telemetry.addData("Left B", colorBotL.blue());
        telemetry.addData("Left A", colorBotL.alpha());
        telemetry.addData("Right R", colorBotR.red());
        telemetry.addData("Right G", colorBotR.green());
        telemetry.addData("Right B", colorBotR.blue());
        telemetry.addData("Right A", colorBotR.alpha());
    }

    public void checkColors(){  //external way of checking, deprecated!
        resultTop = Color.getColor(colorTop.red(),colorTop.green(),colorTop.blue(),colorTop.alpha(),colorTop2.red(),colorTop2.green(),colorTop2.blue(),colorTop2.alpha());
        resultBotL = Color.getColor(colorBotL.red(),colorBotL.green(),colorBotL.blue(),colorBotL.alpha(),colorBotL2.red(),colorBotL2.green(),colorBotL2.blue(),colorBotL2.alpha());
        resultBotR= Color.getColor(colorBotR.red(),colorBotR.green(),colorBotR.blue(),colorBotR.alpha(),colorBotR2.red(),colorBotR2.green(),colorBotR2.blue(),colorBotR2.alpha());
        if(!(resultTop == Color.INVALID) && !(resultBotL == Color.INVALID) && !(resultBotR == Color.INVALID))
            ballsfull = true;
        else
            ballsfull = false;
    }

    public static synchronized Selectioner getInstance(HardwareClass hardwareClass, Telemetry telemetry) {
        if(selectionerInstance == null) {
            selectionerInstance = new Selectioner(hardwareClass, telemetry);
        }

        return selectionerInstance;
    }
}