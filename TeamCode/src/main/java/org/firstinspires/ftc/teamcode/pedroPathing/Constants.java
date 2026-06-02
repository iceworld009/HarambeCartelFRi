package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(16)
            .forwardZeroPowerAcceleration(-33.89)
            .lateralZeroPowerAcceleration(-75.35)//-47
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,
                    0,
                    0.01,
                    0.008
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,
                    0,
                    0.005,
                    0.0005
            ))
            .translationalPIDFSwitch(3.5)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1,
                    0,
                    0.05,
                    0
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    1.2,
                    0,
                    0.05,
                    0
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.01,
                    0,
                    0.00002,
                    0.6,
                    0
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.000005,
                    0.6,
                    0.01
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0004);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("FL")
            .leftRearMotorName("BL")
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(77)
            .yVelocity(61);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.67)//-6.920p
            .strafePodX(1.06)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.95,   // tValueConstraint
            0.06,   // velocityConstraint
            0.1,    // translationalConstraint
            0.009,  // headingConstraint
            50,     // timeout
            1.6,    // brakingStrength
            10,     // BEZIER_CURVE_SEARCH_LIMIT
            21    // brakingStart
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
