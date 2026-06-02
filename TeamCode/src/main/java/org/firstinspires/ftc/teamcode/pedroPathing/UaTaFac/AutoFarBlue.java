package org.firstinspires.ftc.teamcode.pedroPathing.UaTaFac;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Limelight;
import org.firstinspires.ftc.teamcode.Threads.Motors;
import org.firstinspires.ftc.teamcode.Threads.Selectioner;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;


@Autonomous(name = "Auto Far Blue " , group = "Test")
public class AutoFarBlue extends OpMode {
    int target = 0;
    private final Pose startPose = new Pose(133.6, 86.95, Math.toRadians(180));
    private final Pose parkPose = new Pose(100,89,Math.toRadians(180));
    DcMotor FR , FL , BR , BL;
    double shoots = 0;
    private Follower follower;
    int possibleGreenPos = -1;
    Servos servos = null;
    Motors motors = null;
    Limelight limelight = null;
    Selectioner selectioner = null;
    HardwareClass hardwareClass= null;
    Turret turret = null;
    private TelemetryManager telemetryM;

    private Timer pathTimer, opmodeTimer;
    private int pathState;
    boolean isResetTurret = true;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime path = new ElapsedTime();
    double targetAngle, targetPosition,distance;
    private Path scorePreload;
    private PathChain  GrabFirst, Park, GrabBlind;

    int REP = 9000;

    double visionOffset;

    double x,y;
    Pose BotPose;

    double error, targetVelocity = 3850;

    ElapsedTime unload_timer = new ElapsedTime();
    ElapsedTime sleepyyeah = new ElapsedTime();

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, parkPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading());

        Park = follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                .setGlobalDeceleration()
                .setBrakingStart(3)
                .setBrakingStrength(0.4)
                .build();
    }

    void hold(double power){
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(-power);
        BR.setPower(-power);
    }



    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:

                if(sleepyyeah.seconds() > 22) {
                    startPresiune();
                    setPathState(1);
                }
                break;

            case 1:
                if(!follower.isBusy()) {

                    for(int i = 0; i < REP ; i++){
                        errorTelemetry();
                        if(motors.getRampError(targetVelocity)>-60){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    sleep(2000);
                    follower.followPath(Park,true);
                    setPathState(100);
                }
                break;
            case 100:
                if(!follower.isBusy()) {
                    PoseStorage.autoPose = follower.getPose();
                    setPathState(-1);
                }
                break;

        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        updatePosition();
        distance = limelight.getDistanceODMan(x, y, HardwareClass.autoRedScorePoseX, HardwareClass.autoRedScorePoseY);
        error = motors.getRampError(targetVelocity);
        //targetVelocity = getRPM(distance);
        if(target != -1)
            updateTurretFusion();
        autonomousPathUpdate();
        distance = limelight.getDistanceOD(
                follower.getPose().getX(),
                follower.getPose().getY(),
                target
        );
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("RPM: ",motors.getVelocity());
        telemetry.addData("TargetVelocity:",targetVelocity);
        telemetry.addData("Distance to target:",distance);
        telemetry.addData("Timer:", timer.milliseconds());
        telemetry.addData("PreAutoSeek:",possibleGreenPos);
        telemetryM.addData("Rpm",motors.getVelocity());
        telemetry.update();
    }

    public void errorTelemetry(){
        telemetry.clear();
        telemetry.addData("Error:",motors.getRampError(targetVelocity));
        telemetry.update();
    }


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        hardwareClass = HardwareClass.getInstance(hardwareMap);
        servos = Servos.getInstance(hardwareMap,telemetry);
        motors = Motors.getInstance(hardwareMap);
        limelight = Limelight.getInstance(hardwareMap,telemetry);
        limelight.setup();
        limelight.setPipeline(1);
        selectioner = Selectioner.getInstance(hardwareClass,telemetry);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        FL = hardwareClass.FL;
        FR = hardwareClass.FR;
        BL = hardwareClass.BL;
        BR = hardwareClass.BR;
        limelight.startPreSeek();
        unload_timer = new ElapsedTime();

        turret = Turret.getInstance(hardwareMap, telemetry);
        turret.setup();
        turret.resetMotor();
        resetTurret();
        follower.setStartingPose(startPose);
        motors.setRampCoefs();
        distance = limelight.getDistanceOD(follower.getPose().getX(), follower.getPose().getY(),target);
    }


    @Override
    public void init_loop() {
        telemetry.clear();
        telemetry.addData("Seeking:",limelight.possiblePipeline);
        telemetry.clear();
    }


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        possibleGreenPos = limelight.possiblePipeline;
        selectioner.setTagPos(possibleGreenPos);
        limelight.stop();
        turret.powerOn();
        setPathState(0);
        sleepyyeah.reset();
    }
    private void sleep(int delay){
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void shootBall(){
        selectioner.unloadBallsSlow();
        sleep(300);
        shoots++;
    }

    public void startPresiune(){
        hold(0.18);
        selectioner.resetServos();
        motors.intakeOn();
        motors.setCoefsMan(12,0,0,3.5);
        motors.setRampVelocityC((int)(targetVelocity));
    }

    public void stopPresiune(){
        PoseStorage.autoPose = follower.getPose();
        selectioner.unloadBallsSlow();
        motors.setRampVelocityC((int)(0.33 * targetVelocity));
        sleep(100);
        motors.intakeOff();
        hold(0);
    }

    double getHood(double d) {
        if (d < 130) {
            return (d - 100) * (7.0 / 30.0);
        }
        else if (d < 170) {
            return 5 - (d - 130) * (3.0 / 40.0);
        }
        else if (d < 220) {
            return 3 + (d - 170) * (1.6 / 50.0); //  ~4.6 la 220cm
        }
        else {
            return 4.6 + (d - 220) * (2.4 / 130.0); // Rezultă ~7.0 la 350cm
        }
    }

    void resetTurret() {
        target = -1;
        turret.goToPosition(1400);
        sleep(500);
        isResetTurret = true;
        turret.resetMotor();
        turret.goToPosition((HardwareClass.turret_max + HardwareClass.turret_min)/2.0);
        target = 1;
        sleep(500);
        turret.powerOFF();
    }

    void updatePosition(){
        BotPose = follower.getPose();
        x = BotPose.getX();
        y = BotPose.getY();
    }

    void updateTurretFusion() {     //Odometrie instant + corectie limelight
        double dx=0,dy=0;

        limelight.setPipeline(0);
        dx = HardwareClass.autoBlueScorePoseX - x;
        dy = HardwareClass.autoBlueScorePoseY - y + 1;


        double goalAngle = Math.atan2(dy, dx);
        double thetaR = BotPose.getHeading();

        targetAngle = goalAngle - thetaR;
        targetAngle = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle));

        targetPosition = convertToNewRange(
                targetAngle,
                -2*Math.PI/3, 2*Math.PI/3,
                HardwareClass.turret_min, HardwareClass.turret_max
        );

        if (limelight.checkResults()) {
            double tx = limelight.getXPos();
            if(Math.abs(tx)>2)
                visionOffset -= tx * 0.18;
        } else {
            visionOffset *= 0.8;
        }
        targetPosition += visionOffset;

        targetPosition = Math.min(Math.max(targetPosition,HardwareClass.turret_min),HardwareClass.turret_max);
        turret.goToPosition(targetPosition);
    }

    public double convertToNewRange(double value,
                                    double oldMin, double oldMax,
                                    double newMin, double newMax) {
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }


    @Override
    public void stop() {
        motors.rampStop();
        turret.stop();
    }


}