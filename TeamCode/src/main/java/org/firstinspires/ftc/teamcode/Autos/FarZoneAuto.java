package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Initializer;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

@Autonomous
public class FarZoneAuto extends LinearOpMode {
    Initializer init = new Initializer();
    DriveSubsystem chassis;

    public void runOpMode() throws InterruptedException {
        chassis = init.initDrive();

        waitForStart();
        chassis.drive(-0.5, 0, 0);
        sleep(500);
    }
}
