package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "Basic TeleOp Program")
public class TeleOp extends LinearOpMode {
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    public void runOpMode () {
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftBack = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBack = hardwareMap.get(DcMotor.class, "rightBackMotor");

        ChassisSubsystem chassis = new ChassisSubsystem(leftFront, rightFront, leftBack, rightBack);

    while (opModeIsActive()) {
        chassis.drive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                                             gamepad1.right_stick_x);
    }
    }
}
