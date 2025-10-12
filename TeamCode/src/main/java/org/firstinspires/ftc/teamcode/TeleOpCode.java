package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Subsystems.*;

@TeleOp (name = "Basic TeleOp Program")
public class TeleOpCode extends OpMode {
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public ChassisSubsystem chassis;

    @Override
    public void init () {
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftBack = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBack = hardwareMap.get(DcMotor.class, "rightBackMotor");

        ChassisSubsystem chassis = new ChassisSubsystem(leftFront, rightFront, leftBack, rightBack);

    }

    public void loop () {
        chassis.drive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                                             gamepad1.right_stick_x);

    }
}
