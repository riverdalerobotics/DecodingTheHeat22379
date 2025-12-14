package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.TankSubsystem;

@Autonomous
public class FarZoneAuto extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    TankSubsystem chassis;
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        chassis = new TankSubsystem(leftMotor, rightMotor);
        waitForStart();
        chassis.drive(-0.5, 0);
        sleep(500);
    }
}
