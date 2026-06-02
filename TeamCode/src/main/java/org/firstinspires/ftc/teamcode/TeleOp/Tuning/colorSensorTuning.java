package org.firstinspires.ftc.teamcode.TeleOp.Tuning;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@TeleOp
@Config
public class colorSensorTuning extends OpMode {

    public RevColorSensorV3 colorSensor;
    public static String sensorName = "";

    @Override
    public void init(){
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class , sensorName);
    }

    @Override
    public void loop() {
        while(true) {
            telemetry.clearAll();
            telemetry.addData("Rosu", colorSensor.red());
            telemetry.addData("Verde: ", colorSensor.green());
            telemetry.addData("Albastru: ", colorSensor.blue());
            telemetry.addData("Intensity: ", colorSensor.alpha());
            telemetry.addData("Distance:",colorSensor.getDistance(DistanceUnit.CM));

            if (colorSensor.getDistance(DistanceUnit.CM) < 8 && colorSensor.alpha() > 350) ;
            {
                if (colorSensor.green()+100 < (colorSensor.blue() + colorSensor.red()) / 2)
                    telemetry.addLine("Probabil mov");
                else if (colorSensor.green() > colorSensor.red() && colorSensor.green() > colorSensor.blue() && colorSensor.green() > 1000)
                    telemetry.addLine("Probabil verde");
            }
            if (colorSensor.alpha() > 130 && colorSensor.alpha() < 240)
                telemetry.addLine("Bratul este ridicat!");
            else if (colorSensor.alpha() >= 240 && colorSensor.alpha() < 350)
                telemetry.addLine("Hole / uncertain!!!");
            telemetry.update();
        }
    }
}