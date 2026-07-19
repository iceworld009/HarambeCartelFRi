package org.firstinspires.ftc.teamcode.Tuning;


import static org.firstinspires.ftc.teamcode.Utils.Color.GREEN;
import static org.firstinspires.ftc.teamcode.Utils.Color.INVALID;
import static org.firstinspires.ftc.teamcode.Utils.Color.PURPLE;
import static org.firstinspires.ftc.teamcode.Utils.Color.getColor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.Utils.Color;

@TeleOp
@Config
public class colorSensorTuning extends OpMode {

    public RevColorSensorV3 colorSensor, colorSensor2;
    public static String sensorName = "";
    public static String sensorName2 = "";

    @Override
    public void init(){
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class , sensorName);
        this.colorSensor2 = hardwareMap.get(RevColorSensorV3.class , sensorName2);
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

            Color testColor = getColor(colorSensor.red(), colorSensor.green(),colorSensor.blue(),colorSensor.alpha(), colorSensor2.red(), colorSensor2.green(),colorSensor2.blue(),colorSensor2.alpha());
            if(testColor == GREEN)
                telemetry.addLine("Probabil verde");
            if(testColor == PURPLE)
                telemetry.addLine("Probabil mov");
            if(testColor == INVALID)
                telemetry.addLine("Invalid");
            telemetry.update();
        }
    }
}