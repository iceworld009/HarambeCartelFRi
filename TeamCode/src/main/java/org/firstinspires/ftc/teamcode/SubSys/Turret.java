package org.firstinspires.ftc.teamcode.SubSys;
import static org.firstinspires.ftc.teamcode.Utils.GeneralUtils.sleep;

import org.firstinspires.ftc.teamcode.Utils.GeneralUtils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.TurretPID;

public class Turret {

    private final DcMotorEx turretMotor;
    private final Telemetry telemetry;
    private TurretPID pidController;
    private boolean isSetup = false;

    // Tuning parameters
    public static double kp = 0.0055, ki = 0, kd = 0.00003;

    public Turret(HardwareClass hardwareClass, Telemetry telemetry) {
        this.turretMotor = hardwareClass.turret;
        this.telemetry = telemetry;
        this.pidController = new TurretPID(this.turretMotor, telemetry);
    }

    public void setup() {
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidController.setCoefficients(kp, ki, kd);
        pidController.start();
        isSetup = true;
    }

    public int getPosition() {
        return turretMotor.getCurrentPosition();
    }

    public void resetMotor() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void goToPosition(double target) {
        powerOn();
        pidController.setReference(target);
    }

    public void setPID(double p, double i, double d) {
        pidController.setCoefficients(p, i, d);
    }

    public void powerOn() {
        pidController.setPowerEnabled(true);
    }

    public void powerOFF() {
        pidController.setPowerEnabled(false);
    }

    public void stop() {
        isSetup = false;
        if (pidController != null) {
            pidController.stop();
        }
        turretMotor.setPower(0);
    }


    public void resetTurret() {
        goToPosition(1000);
        sleep(1200);
        goToPosition(1200);
        sleep(200);
        resetMotor();
        goToPosition((HardwareClass.turret_max + HardwareClass.turret_min)/2.0);
    }

    public boolean getStatus() {
        return isSetup;
    }
}