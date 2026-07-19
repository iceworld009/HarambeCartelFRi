package org.firstinspires.ftc.teamcode.Tuning;

import static org.firstinspires.ftc.teamcode.Utils.Geometry.getDistanceOD;
import static org.firstinspires.ftc.teamcode.Utils.Geometry.getDistanceODMan;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.SubSys.Limelight;
import org.firstinspires.ftc.teamcode.SubSys.Motors;
import org.firstinspires.ftc.teamcode.SubSys.Selectioner;
import org.firstinspires.ftc.teamcode.SubSys.Servos;
import org.firstinspires.ftc.teamcode.SubSys.Turret;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pizda lu natasha e sexy", group = "Test")
public class LimeLightAutoTest extends OpMode {
    private final Pose startPose = new Pose(0, 0, Math.toRadians(90));
    private final Pose leftPose = new Pose(-5,20,Math.toRadians(90));
    private final Pose rightPose = new Pose(5,20,Math.toRadians(90));
    double targetY;
    private Follower follower;
    private Servos servos;
    private Motors motors;
    private Limelight limelight;
    private Selectioner selectioner;
    private HardwareClass hardwareClass;
    private Turret turret;

    private Timer pathTimer, opmodeTimer;

    // State
    private int pathState;
    private int target = 1;
    private int possibleGreenPos = -1;
    private PathChain grabLeft, grabRight;

    public void buildPaths() {

        grabLeft = follower.pathBuilder()
                .addPath(new BezierLine(startPose, leftPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), leftPose.getHeading())
                .setGlobalDeceleration(0.3)
                .addParametricCallback(0.3, () -> motors.intakeOn())
                .build();

        grabRight = follower.pathBuilder()
                .addPath(new BezierLine(startPose, rightPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), rightPose.getHeading())
                .setGlobalDeceleration(0.3)
                .addParametricCallback(0.3, () -> motors.intakeOn())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                targetY = limelight.getYPos();
                if(targetY>0)
                    setPathState(2);
                if(targetY<=0)
                    setPathState(1);
                break;

            case 1:
                follower.followPath(grabLeft);
                setPathState(3);
                break;

            case 2:
                follower.followPath(grabRight);
                setPathState(3);
                break;

            case 3:
                if(!follower.isBusy()) {
                    telemetry.addData("Ypos:", targetY);
                    break;
                }

        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        hardwareClass = HardwareClass.getInstance(hardwareMap);
        servos = Servos.getInstance(hardwareMap, telemetry);
        motors = Motors.getInstance(hardwareMap);
        limelight = Limelight.getInstance(hardwareMap, telemetry);
        limelight.setup();
        limelight.setPipeline(5);
        selectioner = Selectioner.getInstance(hardwareClass, telemetry);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        motors.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turret = new Turret(hardwareClass, telemetry);
        turret.setup();
        turret.resetTurret();

        follower.setStartingPose(startPose);
        motors.setRampCoefs();
    }

    @Override
    public void init_loop() {
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        selectioner.setTagPos(possibleGreenPos);
        limelight.stop();
        turret.powerOn();
        setPathState(0);
        servos.hoodSetPos(0.5);
    }

    private void sleep(int delayMs) {
        try {
            Thread.sleep(delayMs);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException(e);
        }
    }

    public void stop() {
        motors.rampStop();
        turret.stop();
    }
}
