package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.Discouraged;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

// Do not use this class!
public class MecanumSubsystem extends SubsystemBase {
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightBackMotor;

    public MecanumSubsystem(DcMotor leftFront, DcMotor rightFront,
                            DcMotor leftBack, DcMotor rightBack) {
        this.leftFrontMotor = leftFront;
        this.rightFrontMotor = rightFront;
        this.leftBackMotor = leftBack;
        this.rightBackMotor = rightBack;

        this.leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        this.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(double speed, double strafe, double turn) {
        double rb = speed - turn + strafe;
        double lb = speed + turn - strafe;
        double rf = speed - turn - strafe;
        double lf = speed + turn + strafe;

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

    public void setDrivePower(Pose2d drivePower) {
        double x = drivePower.getX();     // forward/backward
        double y = drivePower.getY();     // strafe
        double heading = drivePower.getHeading(); // rotation

        double frontLeft = x + y + heading;
        double frontRight = x - y - heading;
        double backLeft = x - y + heading;
        double backRight = x + y - heading;

        // Normalize and set motor powers
        double max = Math.max(1.0, Math.max(Math.abs(frontLeft),
                Math.max(Math.abs(frontRight),
                        Math.max(Math.abs(backLeft), Math.abs(backRight)))));

        leftFrontMotor.setPower(frontLeft / max);
        rightFrontMotor.setPower(frontRight / max);
        leftBackMotor.setPower(backLeft / max);
        rightBackMotor.setPower(backRight / max);
    }
}
