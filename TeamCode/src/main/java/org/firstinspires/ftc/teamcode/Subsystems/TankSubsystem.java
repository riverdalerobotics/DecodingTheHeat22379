package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TankSubsystem {

    DcMotor leftMotor;
    DcMotor rightMotor;

    public TankSubsystem(DcMotor leftMotor, DcMotor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    public void drive(double forward, double rotate) {
        double leftPower = forward - rotate;
        double rightPower = forward + rotate;

        double maxPower = Math.abs(leftPower);
        maxPower = Math.max(maxPower, Math.abs(rightPower));

        leftMotor.setPower(leftPower * Math.abs(leftPower) / maxPower);
        rightMotor.setPower(rightPower * Math.abs(rightPower) / maxPower);

    }

}
