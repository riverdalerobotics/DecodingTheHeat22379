package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ChassisSubsystem {
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightBackMotor;
    private double currentSpeed = 0;

    public ChassisSubsystem(DcMotor leftFront, DcMotor rightFront,
                            DcMotor leftBack, DcMotor rightBack) {
        this.leftFrontMotor = leftFront;
        this.rightFrontMotor = rightFront;
        this.leftBackMotor = leftBack;
        this.rightBackMotor = rightBack;
    }

    void drive(double speed, double strafe, double turn) {
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

        currentSpeed = Math.min(max, 1.0);
    }

    // Probably want to have a slew rate function to avoid power overload
    // I don't really know if that is needed
    void slewRateDrive() {

    }
}
