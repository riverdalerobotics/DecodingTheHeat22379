package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

@Autonomous
public class FarZoneAuto extends LinearOpMode {
    DcMotor leftMotor, rightMotor;
    DcMotor leftFront, rightFront, leftBack, rightBack;
    DriveSubsystem chassis;

    public void runOpMode() throws InterruptedException {
        if (!Constants.mecanum) {
            leftMotor = hardwareMap.get(DcMotor.class, Constants.TankLeft);
            rightMotor = hardwareMap.get(DcMotor.class, Constants.TankRight);

            leftMotor.setDirection(DcMotor.Direction.REVERSE);

            chassis = new DriveSubsystem(leftMotor, rightMotor);
        } else {
            leftFront = hardwareMap.get(DcMotor.class, Constants.LeftFront);
            rightFront = hardwareMap.get(DcMotor.class, Constants.RightFront);
            leftBack = hardwareMap.get(DcMotor.class, Constants.LeftBack);
            rightBack = hardwareMap.get(DcMotor.class, Constants.RightBack);

            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);

            chassis = new DriveSubsystem(leftFront, rightFront, leftBack, rightBack);
        }

        waitForStart();
        chassis.drive(-0.5, 0, 0);
        sleep(500);
    }
}
