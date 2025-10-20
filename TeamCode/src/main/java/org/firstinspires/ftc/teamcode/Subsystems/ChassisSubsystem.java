package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class ChassisSubsystem extends SubsystemBase {
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightBackMotor;

    public ChassisSubsystem(DcMotor leftFront, DcMotor rightFront,
                            DcMotor leftBack, DcMotor rightBack) {
        this.leftFrontMotor = leftFront;
        this.rightFrontMotor = rightFront;
        this.leftBackMotor = leftBack;
        this.rightBackMotor = rightBack;
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

}
