    package org.firstinspires.ftc.teamcode.pedroPathing;

    import static org.firstinspires.ftc.teamcode.Utils.MathUtils.convertToNewRange;

    import com.pedropathing.follower.Follower;
    import com.pedropathing.geometry.BezierCurve;
    import com.pedropathing.geometry.BezierLine;
    import com.pedropathing.geometry.Pose;
    import com.pedropathing.paths.Path;
    import com.pedropathing.paths.PathChain;
    import com.pedropathing.util.Timer;
    import com.qualcomm.hardware.lynx.LynxModule;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.teamcode.HardwareClass;
    import org.firstinspires.ftc.teamcode.Threads.Motors;
    import org.firstinspires.ftc.teamcode.Threads.Turret;
    import org.firstinspires.ftc.teamcode.Threads.Selectioner;
    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

    import java.util.List;

    @Autonomous(name = "AutoCloseBlueMk2", group = "Template")
    public class AutoCloseBlueMk2 extends OpMode {

        private Follower follower;
        private Timer pathTimer;
        private int pathState;

        private final Pose startPose = new Pose(14.8, 25.3, Math.toRadians(230.5));
        private final Pose scorePose = new Pose(61.5, 55, Math.toRadians(225));
        private final Pose scorePose1 = new Pose(61.5, 55, Math.toRadians(270));
        private final Pose pickup1_3Pose = new Pose(61.5, 24.5, Math.toRadians(270));
        private final Pose pickup2Pose = new Pose(78, 49, Math.toRadians(270));
        private final Pose aux_2 = new Pose(82, 36, Math.toRadians(270));
        private final Pose pickup2_3Pose = new Pose(85.5, 18.8, Math.toRadians(270));
        private final Pose unloadPose = new Pose(83.5, 12, Math.toRadians(243.5));
        private final Pose unloadPose2 = new Pose(83.8, 12.5, Math.toRadians(243.5));
        private final Pose aux = new Pose(82, 36, Math.toRadians(245));

        private PathChain scorePreload, grabFirst, grabSecond, grabFromRack, grabFromRack2, scoreFromRack, scoreFromRack2, scoreFromRack3;
        HardwareClass hardwareClass = null;
        Turret turret = null;
        Selectioner selectioner = null;
        Motors motors = null;
        Thread updateTurret = null;
        Pose BotPose;
        double x, y, heading;
        int targetRpm = 2600;
        double rpmError;

        public void buildPaths() {
            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                    .addParametricCallback(0, () -> {
                        daiIoaneLaCos();
                    })
                    .setGlobalDeceleration(0.7)
                    .setBrakingStart(20)
                    .setBrakingStrength(5)
                    .build();

            grabFirst = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup2Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                    .addParametricCallback(0.4, () -> {
                        motors.intakeOn();
                    })
                    .addPath(new BezierLine(pickup2Pose, pickup2_3Pose))
                    .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2_3Pose.getHeading())
                    .addPath(new BezierCurve(pickup2_3Pose, aux_2, scorePose1))
                    .addParametricCallback(2.3, () -> {
                        motors.intakeReverse();
                    })
                    .addParametricCallback(2.6, () -> {
                        daiIoaneLaCos();
                    })
                    .addParametricCallback(2.8, () -> {
                        motors.intakeOff();
                    })
                    .setLinearHeadingInterpolation(pickup2_3Pose.getHeading(), scorePose1.getHeading())
                    .setGlobalDeceleration(0.5)
                    .build();

            grabSecond = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose1, pickup1_3Pose))
                    .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1_3Pose.getHeading())
                    .addParametricCallback(0, () -> {
                        motors.intakeOn();
                    })
                    .setGlobalDeceleration(0.7)
                    .setBrakingStart(20)
                    .addPath(new BezierLine(pickup1_3Pose, scorePose1))
                    .setLinearHeadingInterpolation(pickup1_3Pose.getHeading(), scorePose1.getHeading())
                    .addParametricCallback(1.2, () -> {
                        motors.intakeReverse();
                    })
                    .addParametricCallback(1.6, () -> {
                        daiIoaneLaCos();
                    })
                    .setGlobalDeceleration(0.7)
                    .setBrakingStart(20)
                    .addParametricCallback(1.8, () -> {
                        motors.intakeOff();
                    })
                    .setGlobalDeceleration(0.6)
                    .build();

            grabFromRack = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose1, aux_2, unloadPose))
                    .setLinearHeadingInterpolation(scorePose1.getHeading(), unloadPose.getHeading())
                    .addParametricCallback(0.4, () -> {
                        motors.intakeOn();
                    })
                    .setGlobalDeceleration(0.5)
                    .build();

            scoreFromRack = follower.pathBuilder()
                    .addPath(new BezierCurve(unloadPose, aux_2, scorePose1))
                    .setLinearHeadingInterpolation(unloadPose.getHeading(), scorePose1.getHeading())
                    .addParametricCallback(0.4, () -> {
                        motors.intakeReverse();
                        daiIoaneLaCos();
                    })
                    .addParametricCallback(0.9, () -> {
                        motors.intakeOff();
                    })
                    .setGlobalDeceleration(0.5)
                    .build();

            grabFromRack2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose1, aux))
                    .setLinearHeadingInterpolation(scorePose1.getHeading(), aux.getHeading())
                    .addParametricCallback(0.9, () -> {
                        motors.intakeOn();
                    })
                    .addPath(new BezierLine(aux, unloadPose2))
                    .setLinearHeadingInterpolation(aux.getHeading(), unloadPose2.getHeading())
                    .setGlobalDeceleration(0.5)
                    .build();

            scoreFromRack2 = follower.pathBuilder()
                    .addPath(new BezierLine(unloadPose2, scorePose1))
                    .setLinearHeadingInterpolation(unloadPose2.getHeading(), scorePose1.getHeading())
                    .addParametricCallback(0.4, () -> {
                        motors.intakeReverse();
                        daiIoaneLaCos();
                    })
                    .addParametricCallback(0.9, () -> {
                        motors.intakeOff();
                    })
                    .setGlobalDeceleration(0.5)
                    .build();

            scoreFromRack3 = follower.pathBuilder()
                    .addPath(new BezierLine(unloadPose2, scorePose1))
                    .setLinearHeadingInterpolation(unloadPose2.getHeading(), scorePose1.getHeading())
                    .addParametricCallback(0.4, () -> {
                        motors.intakeReverse();
                        daiIoaneLaCos();
                    })
                    .addParametricCallback(0.9, () -> {
                        motors.intakeOff();
                    })
                    .setGlobalDeceleration(0.5)
                    .build();
        }

        public void autonomousPathUpdate() {
            switch (pathState) {
                case 0:
                    follower.followPath(scorePreload, true);
                    setPathState(1);
                    break;
                case 1:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                        potolYourself();
                        setPathState(2);
                    }
                    break;

                case 2:
                    follower.followPath(grabFirst, true);
                    setPathState(3);
                    break;
                case 3:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                        potolYourself();
                        setPathState(4);
                    }
                    break;

                case 4:
                    follower.followPath(grabFromRack, true);
                    setPathState(5);
                    break;
                case 5:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                        sleep(1000);
                        setPathState(6);
                    }
                    break;

                case 6:
                    follower.followPath(scoreFromRack, true);
                    setPathState(7);
                    break;
                case 7:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                        potolYourself();
                        setPathState(8);
                    }
                    break;

                case 8:
                    follower.followPath(grabSecond, true);
                    setPathState(9);
                    break;
                case 9:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                        potolYourself();
                        setPathState(10);
                    }
                    break;

                case 10:
                    follower.followPath(grabFromRack2, true);
                    setPathState(11);
                    break;
                case 11:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                        sleep(1000);
                        setPathState(12);
                    }
                    break;

                case 12:
                    follower.followPath(scoreFromRack2, true);
                    setPathState(13);
                    break;
                case 13:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                        potolYourself();
                        setPathState(14);
                    }
                    break;

                case 14:
                    follower.followPath(grabFromRack2, true);
                    setPathState(15);
                    break;
                case 15:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                        sleep(1000);
                        setPathState(16);
                    }
                    break;

                case 16:
                    follower.followPath(scoreFromRack2, true);
                    setPathState(17);
                    break;
                case 17:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                        potolYourself();
                        setPathState(18);
                    }
                    break;

                case 18:
                    follower.followPath(grabFromRack2, true);
                    setPathState(19);
                    break;
                case 19:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                        sleep(1000);
                        setPathState(20);
                    }
                    break;

                case 20:
                    follower.followPath(scoreFromRack3, true);
                    setPathState(21);
                    break;
                case 21:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                        potolYourself();
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
        public void init() {
            List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            hardwareClass = HardwareClass.getInstance(hardwareMap);
            turret = Turret.getInstance(hardwareMap, telemetry);
            selectioner = Selectioner.getInstance(hardwareClass, telemetry);
            motors = Motors.getInstance(hardwareMap);
            pathTimer = new Timer();
            follower = Constants.createFollower(hardwareMap);
            buildPaths();
            follower.setStartingPose(startPose);
            if (!turret.getStatus()) {
                turret.setup();
            }
            turret.resetTurret();
            selectioner.resetServos();
        }

        @Override
        public void start() {
            turret.powerOn();
            startUpdate();
            setPathState(0);
        }

        @Override
        public void loop() {
            follower.update();
            updatePosition();
            autonomousPathUpdate();
            rpmError = motors.getRampError(targetRpm);
            motors.setRampCoefs(hardwareMap.voltageSensor.iterator().next().getVoltage(), -0.5);
            telemetry.addData("Path State", pathState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

        void updatePosition() {
            BotPose = follower.getPose();
            x = BotPose.getX();
            y = BotPose.getY();
        }

        void updateTurretFusion() {
            double dx = HardwareClass.autoBlueScorePoseX - x;
            double dy = HardwareClass.autoBlueScorePoseY - y;

            double goalAngle = Math.atan2(dy, dx);
            double thetaR = BotPose.getHeading();

            double targetAngle = goalAngle - thetaR;
            targetAngle = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle));

            double targetPosition = convertToNewRange(
                    targetAngle,
                    -2 * Math.PI / 3, 2 * Math.PI / 3,
                    HardwareClass.turret_min, HardwareClass.turret_max
            );

            targetPosition = Math.min(Math.max(targetPosition, HardwareClass.turret_min), HardwareClass.turret_max);
            turret.goToPosition(targetPosition);
        }

        void daiIoaneLaCos() {
            motors.setRampCoefs();
            motors.setRampVelocityC(targetRpm);
        }

        void potolYourself() {
            PoseStorage.autoPose = follower.getPose();
            selectioner.unloadBallsSlow();
            sleep(200);
            motors.setRampVelocityC((int) (targetRpm * 0.4));
        }

        public void startUpdate() {
            boolean running = true;
            if (updateTurret == null || !updateTurret.isAlive()) {
                updateTurret = new Thread(() -> {
                    while (running) {
                        updatePosition();
                        updateTurretFusion();
                        sleep(10);
                    }
                });
                updateTurret.start();
            }
        }

        private void sleep(int delay) {
            try {
                Thread.sleep(delay);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        @Override
        public void stop() {
            motors.rampStop();
            turret.stop();
        }
    }