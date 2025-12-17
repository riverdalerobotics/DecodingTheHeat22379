package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {
    // Put all constants in here
    // i.e. any number that will not change and may need to be referenced a lot
    // Right now, all numbers in here are placeholders and need to be changed
    @Configurable
    public static class CameraConstants {
        public static double x = 0;
        public static double y = 0;
        public static double z = 0;
        public static double yaw = 0;
        public static double pitch = 0;
        public static double roll = 0;
    }

    @Configurable
    public static class IntakeConstants {
        public static double forward = -1;
        public static double backward = 1;
    }

    public static String Shooter = "S";
    public static String Intake = "I";
    public static String TankLeft = "leftMotor";
    public static String TankRight = "rightMotor";
    public static String LeftFront = "leftFront";
    public static String RightFront = "rightFront";
    public static String LeftBack = "leftBack";
    public static String RightBack = "rightBack";
    public static String Indexer = "gatekeeper";

    public static boolean mecanum = false;
    double[][] ShootingArea = {
        {0, 0},
        {0, 3657},
        {3657, 3657},
        {3657, 0}
    }; // This is the whole field, find actual numbers

    public static final double servoClosedPosition = 1;
    public static final double servoOpenPosition = 0.8;
}
