package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class BlueSwerveConst {
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4.25)
            .strafePodX(-.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .xVelocity(82)
            .yVelocity(82)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-81)
            .lateralZeroPowerAcceleration(-81)
            .centripetalScaling(.00001)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07, 0, 0.004, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(.6, 0, 0.03, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.3, 0, 0.15, 0.06, 0.1))
            .automaticHoldEnd(false)
            .turnHeadingErrorThreshold(Math.toRadians(5))
            .mass(10.9);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 20, 2);

    public static Follower createFollower(HardwareMap hardwareMap, Gamepad gamepad, double color) {

        return new FollowerBuilder(followerConstants, hardwareMap)
                .setDrivetrain(new SwervePedro(hardwareMap, driveConstants, gamepad, color))
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }


}
