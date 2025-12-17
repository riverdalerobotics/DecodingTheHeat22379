package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

public class DriveSubsystem {

    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightBackMotor;

    public DriveSubsystem(DcMotor leftMotor, DcMotor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    public DriveSubsystem(DcMotor leftFront, DcMotor rightFront,
                            DcMotor leftBack, DcMotor rightBack) {
        this.leftFrontMotor = leftFront;
        this.rightFrontMotor = rightFront;
        this.leftBackMotor = leftBack;
        this.rightBackMotor = rightBack;

        this.leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        this.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(double forward, double rotate, double strafe) {
        if (!Constants.mecanum) {
            double leftPower = forward - rotate;
            double rightPower = forward + rotate;

            double maxPower = Math.abs(leftPower);
            maxPower = Math.max(maxPower, Math.abs(rightPower));

            leftMotor.setPower(leftPower * Math.abs(leftPower) / maxPower);
            rightMotor.setPower(rightPower * Math.abs(rightPower) / maxPower);
        } else {
            double rb = forward - rotate + strafe;
            double lb = forward + rotate - strafe;
            double rf = forward - rotate - strafe;
            double lf = forward + rotate + strafe;

            // If max is too big, we need to adjust
            double max = Math.max(Math.abs(rf), Math.abs(rb));
            max = Math.max(max,Math.abs(lf));
            max = Math.max(max, Math.abs(lb));

            if (max > 1.0) {
                rb /= max; lb /= max; rf /= max; lf /= max;
            }
            leftFrontMotor.setPower(lf);
            leftBackMotor.setPower(lb);
            rightFrontMotor.setPower(rf);
            rightBackMotor.setPower(rb);
        }
    }
}
