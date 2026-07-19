package org.firstinspires.ftc.teamcode.SubSys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.HardwareClass;

public class Motors {

    private DcMotorEx ramp,ramp2;
    public DcMotor intakeMotor;
//    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(12, 0, 0, 5.8);
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(34, 0, 0, 2);
    private static Motors instance;
    public double targetVelocity = 0;
    public double nominalVoltage = 13.0;
    private Motors(HardwareClass hw) {
        ramp = hw.ramp;
        ramp2 = hw.ramp2;
        intakeMotor = hw.intakeMotor;

        ramp2.setDirection(DcMotorSimple.Direction.FORWARD);
        ramp.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intakeOn() {
        intakeMotor.setPower(0.9);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    public void intakeReverse() {
        intakeMotor.setPower(-0.9);
    }

    public void setRampVelocityC(int velocity) {
        ramp.setVelocity(velocity);
        ramp2.setVelocity(velocity);
        targetVelocity = velocity;
    }
    public double getVelocity() {
        return (ramp.getVelocity()*60/28); // 6000 rpm motor
    }

    public double getRampError(){
        return getVelocity() - targetVelocity;
    }

    public void rampStop() {
        ramp.setPower(0);
        ramp2.setPower(0);
    }

    public void setRampCoefs(){
        ramp.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d, MOTOR_VELO_PID.f));

        ramp2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,  MOTOR_VELO_PID.f));
        ramp2.setDirection(DcMotorSimple.Direction.FORWARD);
        ramp.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setRampCoefs(double voltage){
        double nominalVoltage = 13.0;
        ramp.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d, MOTOR_VELO_PID.f * (nominalVoltage/voltage)));

        ramp2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,  MOTOR_VELO_PID.f * (nominalVoltage/voltage)));
        ramp2.setDirection(DcMotorSimple.Direction.FORWARD);
        ramp.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setRampCoefs(double voltage, double boost){
        double nominalVoltage = 12.0;
        ramp.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d, MOTOR_VELO_PID.f * (nominalVoltage/voltage)+boost));

        ramp2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,  MOTOR_VELO_PID.f * (nominalVoltage/voltage)+boost));
        ramp2.setDirection(DcMotorSimple.Direction.FORWARD);
        ramp.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void setCoefsMan(double p, double i, double d, double f, double voltage){
        ramp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ramp2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ramp.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(p, i, d,f * (nominalVoltage / voltage )));
        ramp2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(p, i, d,f * (nominalVoltage / voltage)));
        ramp2.setDirection(DcMotorSimple.Direction.FORWARD);
        ramp.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setCoefsMan(double p, double i, double d, double f){
        ramp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ramp2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ramp.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(p, i, d,f));
        ramp2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(p, i, d,f));
        ramp2.setDirection(DcMotorSimple.Direction.FORWARD);
        ramp.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static synchronized Motors getInstance(HardwareMap hw) {
        if (instance == null) {
            instance = new Motors(HardwareClass.getInstance(hw));
        }
        return instance;
    }
}