package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Constants {
    // Put all constants in here
    // i.e. any number that will not change and may need to be referenced a lot

    // Right now, all numbers in CameraConstants are placeholders and need to be changed
    @Configurable
    public static class CameraConstants {
        public static double P = 0.02;
        public static double verticalFOV = 42;
        public static double horizontalFOV = 54.5;
        public static double verticalPixelCount = 480;
        public static double horizontalPixelCount = 640;
        public static double vertPixelPerDeg = verticalPixelCount / verticalFOV;
        public static double horPixelPerDeg = horizontalPixelCount / horizontalFOV;
        public static double pitch = Math.toRadians(16);
        public static double height = 17.4;
        public static double apriltagHeight = 30*2.54;
    }

    @Configurable
    public static class IntakeConstants {
        public static double forward = -1;
        public static double backward = 1;

        public static String Intake = "I";
    }

    @Configurable
    public static class DriveConstants {
        public static String TankLeft = "leftMotor";
        public static String TankRight = "rightMotor";
        public static String LeftFront = "leftFront";
        public static String RightFront = "rightFront";
        public static String LeftBack = "leftBack";
        public static String RightBack = "rightBack";
        public static double autoTurnP = 0.01;
        public static double rotationStallSpeed = 0.12;
        public static double forwardP = 0.001;
        public static double forwardStallSpeed = 0.2;
        public static double angleThreshold = 30;
    }

    @Configurable
    public static class ShooterConstants {
        public static String Shooter = "S";
        public static String Indexer = "gatekeeper";

        public static final double servoClosedPosition = 0.95;
        public static final double servoOpenPosition = 0.8;
        public static double distanceModifier = 0.0012;
        public static double batteryModifier = 0.03;
        public static double startingPower = 0.83;
    }

    public static String Limelight = "limelight";
    public static String IMU = "imu";

    public static boolean mecanum = true;
    double[][] ShootingArea = {
        {0, 0},
        {0, 3657},
        {3657, 3657},
        {3657, 0}
    }; // This is the whole field, find actual numbers

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5); // NOT CORRECT!
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(DriveConstants.RightFront)
            .leftFrontMotorName(DriveConstants.LeftFront)
            .rightRearMotorName(DriveConstants.RightBack)
            .leftRearMotorName(DriveConstants.LeftBack)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName(DriveConstants.RightFront)
            .rightRearMotorName(DriveConstants.RightBack)
            .leftRearMotorName(DriveConstants.LeftBack)
            .leftFrontMotorName(DriveConstants.LeftFront)
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            .robotWidth(15)
            .robotLength(15)
            .forwardTicksToInches((-0.0318983-0.03208737-0.31756)/3)
            .strafeTicksToInches(0.06400986241942351)
            .turnTicksToInches(0.0019516);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .driveEncoderLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
