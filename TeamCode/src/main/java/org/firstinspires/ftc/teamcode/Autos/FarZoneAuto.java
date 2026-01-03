package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OI;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

@Autonomous
public class FarZoneAuto extends LinearOpMode {
    DriveSubsystem chassis;

    public void runOpMode() throws InterruptedException {

        waitForStart();
        chassis.drive(-0.5, 0, 0);
        sleep(500);
    }
}
