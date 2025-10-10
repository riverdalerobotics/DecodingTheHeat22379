package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Mecanum {
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightBackMotor;

    public Mecanum (DcMotor leftFront, DcMotor rightFront,
                    DcMotor leftBack, DcMotor rightBack) {
        this.leftFrontMotor = leftFront;
        this.rightFrontMotor = rightFront;
        this.leftBackMotor = leftBack;
        this.rightBackMotor = rightBack;
    }

    void drive(boolean forward, boolean strafe, boolean turn) {
        // Put in the robot oriented driving here.
        int lf = 0;
        int rf = 0;
        int lb = 0;
        int rb = 0;
    }
}
