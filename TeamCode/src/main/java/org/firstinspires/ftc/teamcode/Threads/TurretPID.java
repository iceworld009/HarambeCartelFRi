package org.firstinspires.ftc.teamcode.Threads;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;

public class TurretPID implements Runnable{

    private volatile boolean running = false;
    private String threadName = null;
    private String motorName = null;
    private Telemetry telemetry;
    private Thread thread = null;
    private static TurretPID single_instance = null;

    private double Kp, Ki, Kd;
    private double reference, current = 0;
    private double integralSum, derivative;
    private double error, lastError;
    private DcMotorEx motor1;

    double repetitions=0;

    private int activePower = 1;
    int stop = 1;

    /*
    Explicit 'this' to state object type clearly
     */

    /**
     * @param motor1 - the motor to which the PID controller affects
     * @param telemetry - telemtry object for logging
     */
    public TurretPID(DcMotorEx motor1, String motorName, String threadName, Telemetry telemetry){
        this.motor1 = motor1;
        this.motorName = motorName;
        this.threadName = threadName;
        this.telemetry = telemetry;
    }

    public TurretPID(DcMotorEx motor1,
                      double kp,
                      double ki,
                      double kd,
                      double reference,
                      String motorName,
                      String threadName,
                      Telemetry telemetry){
        this.motor1 = motor1;
        this.Kp = kp;
        this.Ki = ki;
        this.Kd = kd;
        this.motorName = motorName;
        this.threadName = threadName;
        this.telemetry = telemetry;
    }
    public boolean start(){
        try {
            if (this.thread == null || !this.thread.isAlive()) {
                //telemetry.addLine("Trying to start");
                //telemetry.update();
                this.thread = new Thread(this);
                this.running = true;
                this.current = motor1.getCurrentPosition();
                this.repetitions=0;
                this.thread.start();

                return true;
            } else{
                return false;
            }
        }catch (IllegalThreadStateException e){
            //this.telemetry.addData(this.threadName + " -> thread start error: ",e);
            //telemetry.update();
            return false;
        }
    }

    public synchronized void resetPID() {
        integralSum = 0;
        lastError = 0;
        error = 0;
        repetitions = 0;
        stop = 1;
    }


    public boolean status(){
        return running;
    }

    public synchronized boolean stop(){
        running = false;
        stop = 0;

        if (motor1 != null) {
            motor1.setPower(0);
        }

        if (thread != null) {
            try {
                thread.join(100);
            } catch (InterruptedException ignored) {}
        }

        thread = null;
        resetPID();
        return true;
    }


    /**
     * @param p - kp
     * @param i - ki
     * @param d - kd
     */
    public void setCoefficients(double p, double i, double d){
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
    }

    /**
     * @param reference - desired position
     */
    public void setReference(double reference){
        this.reference = reference;
    }

    public void delay(int sec) throws  InterruptedException{
        this.thread.wait(sec);
    }

    public void stopPID(){
        stop = 0;
    }

    public void startPID(){
        stop = 1;
    }

    FtcDashboard ftdb = FtcDashboard.getInstance();

    @Override
    public void run() {
        // short amount of time resets in every loop so every point of the
        while(running) {
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while ((Math.abs((int)(error)) > 10 || repetitions < 40) && stop == 1) {

                current = (int)(motor1.getCurrentPosition());

                error = reference - current;
                derivative = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());


                double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                double power = Math.max(-0.7, Math.min(0.7, output * activePower));
                motor1.setPower(power);


               //telemetry.addData(motorName + " target", reference);
               //telemetry.addData(motorName + " pos", current);
               //telemetry.addData(motorName + " error", error);
//
//                TelemetryPacket tp = new TelemetryPacket();
//                tp.put("pos", current);
//                tp.put("ref", reference);
//                tp.put("error", error);
//                tp.put("targetAngle: ",reference);
//                ftdb.sendTelemetryPacket(
//                        tp
//                );

                //telemetry.update();
                lastError = error;
                timer.reset();
                //try { Thread.sleep(10); } catch (InterruptedException ignored) {}  // 10 mill pentru siguranta
            }
            current = motor1.getCurrentPosition();
//            telemetry.addData(motorName + " pos", current);
//            telemetry.update();
            repetitions++;
        }
    }

    public double convertToNewRange(double value,
                                    double oldMin, double oldMax,
                                    double newMin, double newMax) {
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }

    public void activetePower(){
        activePower = 1;
    }

    public void deactivatePower(){
        activePower = 0;
    }

    public static synchronized TurretPID getInstance(DcMotorEx motor1, String motorName, String threadName, Telemetry telemetry){
        if(single_instance == null){
            single_instance = new TurretPID(motor1,motorName  , threadName,telemetry);
        }
        return single_instance;
    }

}