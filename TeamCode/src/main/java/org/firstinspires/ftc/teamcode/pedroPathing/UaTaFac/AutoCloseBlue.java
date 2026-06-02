package org.firstinspires.ftc.teamcode.pedroPathing.UaTaFac;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Limelight;
import org.firstinspires.ftc.teamcode.Threads.Motors;
import org.firstinspires.ftc.teamcode.Threads.Selectioner;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;


@Autonomous(name = "Auto Close Blue " , group = "Test")
public class AutoCloseBlue extends OpMode {
    int target=0;
    private final Pose startPose = new Pose(14.8, 25.3, Math.toRadians(230.5)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(46.1, 45  ,Math.toRadians(225)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose1 = new Pose(64,49,Math.toRadians(270)); // scorePose 1 doar cu turreta
    private final Pose pickup1_3Pose = new Pose(63.1, 24.5, Math.toRadians(270)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(78, 49   , Math.toRadians(270)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose aux_2 = new Pose(82, 36   , Math.toRadians(270)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2_3Pose = new Pose(85.5,18,Math.toRadians(270));
    private final Pose parkPose = new Pose(40.7, 51, Math.toRadians(215)); // Park // Park
    private final Pose unloadPose = new Pose(82.8,11,Math.toRadians(248));
    private final Pose unloadPose2 = new Pose(83.1,11,Math.toRadians(245));
    private final Pose aux = new Pose (82 , 36  , Math.toRadians(245));
    DcMotor FR , FL , BR , BL;
    double shoots = 0;
    private Follower follower;
    int possibleGreenPos = -1;
    private final ElapsedTime delay = new ElapsedTime();
    Servos servos = null;
    Motors motors = null;
    Limelight limelight = null;
    Selectioner selectioner = null;
    HardwareClass hardwareClass= null;
    Turret turret = null;
    private TelemetryManager telemetryM;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    boolean isResetTurret = true;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime path = new ElapsedTime();
    double targetAngle, targetPosition,distance;
    private Path scorePreload;
    private PathChain grabPickup1, grabPickup1_3, grabPickup2, grabPickup2_3,scorePickup2,scorePickup3, preload,scorePickup1, Park,rPath1;

    private PathChain unload1,unload2;
    private PathChain ScorePreload , GrabFirst, GrabFromRack, ScoreFromRack, GrabSecond, ScoreFromRack2, ScoreFromRack3,GrabFromRack2;

    int REP = 3000;

    double visionOffset;

    double x,y;
    Pose BotPose;

    double error, targetVelocity = 2450;

    ElapsedTime unload_timer = new ElapsedTime();

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        ScorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0,() -> {
                    selectioner.resetServos();
                    startPresiune();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.4)
                .build();


        GrabFirst = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(pickup2Pose, pickup2_3Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2_3Pose.getHeading())
                .addPath(new BezierLine(pickup2_3Pose, aux_2))
                .setLinearHeadingInterpolation(pickup2_3Pose.getHeading(), aux_2.getHeading())
                .addParametricCallback(0.5,() -> {
                    motors.intakeReverse();
                })
                .addPath(new BezierLine(aux_2, scorePose1))
                .setLinearHeadingInterpolation(aux_2.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0.1,() -> {
                    startPresiune();
                })
                .addParametricCallback(0.95,() -> {
                    motors.intakeOff();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.4)
                .build();

        GrabSecond = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, pickup1_3Pose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1_3Pose.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(pickup1_3Pose, scorePose1))
                .setLinearHeadingInterpolation(pickup1_3Pose.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0.1,() -> {
                    startPresiune();
                })
                .addParametricCallback(0.65,() -> {
                    motors.intakeReverse();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.5)
                .build();

        GrabFromRack = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose1, aux_2, unloadPose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), unloadPose.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addParametricCallback(1,() -> {
                    unload_timer.reset();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.5)
                .build();

        GrabFromRack2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, aux))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), aux.getHeading())
                .addPath(new BezierLine(aux, unloadPose2))
                .setLinearHeadingInterpolation(aux.getHeading(), unloadPose2.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeOn();
                    selectioner.resetServos();

                })
                .addParametricCallback(0.9,() -> {
                    unload_timer.reset();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.5)
                .build();


        ScoreFromRack = follower.pathBuilder()
                .addPath(new BezierCurve(unloadPose, aux_2,scorePose1))
                .setLinearHeadingInterpolation(unloadPose.getHeading(), aux_2.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeReverse();
                })
                .setLinearHeadingInterpolation(aux_2.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0,() -> {
                    startPresiune();
                })
                .addParametricCallback(0.45,() -> {
                    motors.intakeReverse();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.5)
                .build();

        ScoreFromRack2 = follower.pathBuilder()
                .addPath(new BezierLine(unloadPose2, scorePose1))
                .setLinearHeadingInterpolation(unloadPose2.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeReverse();
                    startPresiune();
                })
                .addParametricCallback(0.3,() -> {
                    motors.intakeReverse();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.5)
                .build();

        ScoreFromRack3 = follower.pathBuilder()
                .addPath(new BezierLine(unloadPose2, parkPose))
                .setLinearHeadingInterpolation(unloadPose2.getHeading(), parkPose.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeReverse();
                    startPresiune();
                })
                .addParametricCallback(0.3,() -> {
                    motors.intakeReverse();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.5)
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
                follower.followPath(ScorePreload,true);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()) {
                    motors.intakeOn();
                    for(int i = 0; i < REP ; i++){
                        errorTelemetry();
                        if(motors.getRampError(targetVelocity)>-110){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(GrabFirst,true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    motors.intakeOn();
                    for(int i = 0; i < REP ; i++){
                        errorTelemetry();
                        if(motors.getRampError(targetVelocity)>-80){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(GrabFromRack, true);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()) {
                    if(unload_timer.milliseconds() > 2300){
                        sleep(1200);
                        setPathState(7);
                    }
                }
                break;

            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(ScoreFromRack,true);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    motors.intakeOn();
                    for(int i = 0; i < REP ; i++){
                        errorTelemetry();
                        if(motors.getRampError(targetVelocity)>-80){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(9);
                }
                break;

            case 9:

                if(!follower.isBusy()) {
                    follower.followPath(GrabSecond,true);
                    setPathState(10);
                }
                break;

            case 10:
                if(!follower.isBusy()) {
                    startPresiune();
                    motors.intakeOn();
                    for(int i = 0; i < REP ; i++){
                        errorTelemetry();
                        if(motors.getRampError(targetVelocity)>-80){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(16);
                }
                break;

            case 11:

                if(!follower.isBusy()) {
                    follower.followPath(GrabFromRack2, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    if(unload_timer.milliseconds() > 2500){
                        setPathState(14);
                    }
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    follower.followPath(ScoreFromRack2,true);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    PoseStorage.autoPose = follower.getPose();
                    motors.intakeOn();
                    for(int i = 0; i < REP ; i++){
                        errorTelemetry();
                        if(motors.getRampError(targetVelocity)>-80){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(16);
                }
                break;
            case 16:

                if(!follower.isBusy()) {
                    follower.followPath(GrabFromRack2,true);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    if(unload_timer.milliseconds() > 1150){
                        setPathState(18);
                    }
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    follower.followPath(ScoreFromRack3,true);
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    PoseStorage.autoPose = follower.getPose();
                    motors.intakeOn();
                    for(int i = 0; i < REP && opmodeTimer.getElapsedTimeSeconds() < 29.1; i++){
                        errorTelemetry();
                        if(motors.getRampError(targetVelocity)>-65){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(20);
                }
                break;
            case 20:
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
        motors.intakeOn();
        motors.setCoefsMan(12,0,0,3.5);
        motors.setRampVelocityC((int)(targetVelocity));
    }

    public void stopPresiune(){
        PoseStorage.autoPose = follower.getPose();
        //selectioner.shootOnAT(0);
        //selectioner.shootOnAT(0,40);
        selectioner.unloadBalls();
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
        turret.goToPosition(turret.getPosition());
        target = 1;
        sleep(100);
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
        dy = HardwareClass.autoBlueScorePoseY - y;


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
            if(Math.abs(tx)>1.2)
                visionOffset -= tx * 0.20;
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