package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretPID implements Runnable {

    private volatile boolean running = false;
    private final DcMotorEx motor;
    private final Telemetry telemetry;
    private Thread thread = null;

    private volatile double Kp, Ki, Kd;
    private volatile double reference = 0;
    private volatile boolean powerEnabled = true;

    private double integralSum = 0;
    private double lastError = 0;

    public TurretPID(DcMotorEx motor, Telemetry telemetry) {
        this.motor = motor;
        this.telemetry = telemetry;
    }

    public synchronized void start() {
        if (thread == null || !thread.isAlive()) {
            running = true;
            integralSum = 0;
            lastError = 0;
            thread = new Thread(this, "Turret-PID-Thread");
            thread.start();
        }
    }

    public synchronized void stop() {
        running = false;
        if (thread != null) {
            thread.interrupt();
            try {
                thread.join(150); // Wait for it to die ;)
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
            thread = null;
        }
        if (motor != null) {
            motor.setPower(0);
        }
    }

    public void setCoefficients(double p, double i, double d) {
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
    }

    public void setReference(double reference) {
        this.reference = reference;
    }

    public void setPowerEnabled(boolean enabled) {
        this.powerEnabled = enabled;
    }

    @Override
    public void run() {
        ElapsedTime timer = new ElapsedTime();
        lastError = 0;
        integralSum = 0;

        while (running && !Thread.currentThread().isInterrupted()) {
            double current = motor.getCurrentPosition();
            double error = reference - current;

            double dt = timer.seconds();
            if (dt < 0.0001) dt = 0.0001;

            double derivative = (error - lastError) / dt;

            // Basic windup guard
            if (Math.abs(error) < 300) {
                integralSum += error * dt;
            } else {
                integralSum = 0;
            }

            double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            if (powerEnabled) {
                double power = Math.max(-0.7, Math.min(0.7, output));
                motor.setPower(power);
            } else {
                motor.setPower(0);
            }

            lastError = error;
            timer.reset();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        motor.setPower(0);
    }
}