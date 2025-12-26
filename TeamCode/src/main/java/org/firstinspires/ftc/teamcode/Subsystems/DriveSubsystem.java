package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.*;

public class DriveSubsystem extends SubsystemBase {

    public DcMotor leftMotor, rightMotor;
    public DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    public DcMotor[] motors;
    public IMU imu;
    double currentYaw;
    double slowMode = 1;
    public String color = "Blue";

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
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors = new DcMotor[]{leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor};

        this.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    public void setUpIMU(IMU imu) {
        this.imu = imu;
        imu.resetYaw();
        currentYaw = 0;
    }

    public void drive(double forward, double rotate, double strafe) {
        if (!Constants.mecanum) {
            double leftPower = forward - strafe;
            double rightPower = forward + strafe;

            double maxPower = Math.abs(leftPower);
            maxPower = Math.max(maxPower, Math.abs(rightPower));

            leftMotor.setPower(leftPower * Math.abs(leftPower) / maxPower);
            rightMotor.setPower(rightPower * Math.abs(rightPower) / maxPower);
        } else {
            if (rotate == 0) {
                // Here we need to add a rotation factor based on how much we deviated from our angle
                double deltaYaw = imu.getRobotYawPitchRollAngles().getYaw() - currentYaw;
                if (Math.abs(deltaYaw) <= DriveConstants.angleThreshold) {
                    rotate = deltaYaw * DriveConstants.autoTurnP;
                }
            } else {
                currentYaw = imu.getRobotYawPitchRollAngles().getYaw();
            }
            double rb = forward + strafe - rotate;
            double lb = forward - strafe + rotate;
            // Actually should be lf
            double rf = forward - strafe - rotate;
            // Actually should be rf;
            double lf = forward + strafe + rotate;

            // If max is too big, we need to adjust
            double max = Math.max(Math.abs(rf), Math.abs(rb));
            max = Math.max(max,Math.abs(lf));
            max = Math.max(max, Math.abs(lb));

            if (max > 1.0) {
                rb /= max; lb /= max; rf /= max; lf /= max;
            }
            leftFrontMotor.setPower(lf / slowMode);
            leftBackMotor.setPower(lb / slowMode);
            rightFrontMotor.setPower(rf / slowMode);
            rightBackMotor.setPower(rb / slowMode);
        }
    }

    public void toggleSlowMode() {
        if (slowMode == 1) {
            slowMode = 2;
        } else {
            slowMode = 1;
        }
    }

    public void forwardByTicks(int ticks) {
        int[] curTicks = {0, 0, 0, 0};
        for (int i = 0; i < 4; i++) {
            curTicks[i] = motors[i].getCurrentPosition();
        }

        while (true) {
            int totalTicksMoved = 0;
            for (int i = 0; i < 4; i++) {
                totalTicksMoved += motors[i].getCurrentPosition() - curTicks[i];
            }
            if (Math.abs(totalTicksMoved/4) >= Math.abs(ticks)) {
                holdHeading(200);
                break;
            } else {
                double error = (double)ticks - (double)(totalTicksMoved)/4.0;
                double forwardPower = error*DriveConstants.forwardP + Math.copySign(DriveConstants.forwardStallSpeed, error);
                forwardPower = Math.max(-0.5, Math.min(0.5, forwardPower));

                drive(forwardPower, 0, 0);
            }
        }
    }

    public void forwardByTicks(int ticks, double power) {
        int[] curTicks = {0, 0, 0, 0};
        for (int i = 0; i < 4; i++) {
            curTicks[i] = motors[i].getCurrentPosition();
        }

        while (true) {
            int totalTicksMoved = 0;
            for (int i = 0; i < 4; i++) {
                totalTicksMoved += motors[i].getCurrentPosition() - curTicks[i];
            }
            if (Math.abs(totalTicksMoved/4) >= Math.abs(ticks)) {
                holdHeading(200);
                break;
            } else {
                double error = (double)ticks - (double)(totalTicksMoved)/4.0;
                double forwardPower = error*DriveConstants.forwardP + Math.copySign(DriveConstants.forwardStallSpeed, error);
                forwardPower = Math.max(-power, Math.min(power, forwardPower));

                drive(forwardPower, 0, 0);
            }
        }
    }


    public void strafeByTicks(int ticks) {
        // Record the starting tick count for each motor.
        int[] startTicks = {
                leftFrontMotor.getCurrentPosition(),
                rightFrontMotor.getCurrentPosition(),leftBackMotor.getCurrentPosition(),
                rightBackMotor.getCurrentPosition()
        };

        // A small constant power to overcome static friction.
        // The sign is positive for right strafe, negative for left.

        while (true) {
            // For strafing, some motors move forward and others backward.
            int lfTicksMoved = leftFrontMotor.getCurrentPosition() - startTicks[0];
            int rfTicksMoved = rightFrontMotor.getCurrentPosition() - startTicks[1];
            int lbTicksMoved = leftBackMotor.getCurrentPosition() - startTicks[2];
            int rbTicksMoved = rightBackMotor.getCurrentPosition() - startTicks[3];

            // lf: +ticks, rf: -ticks, lb: -ticks, rb: +ticks
            double averageTicksMoved = (lfTicksMoved - rfTicksMoved - lbTicksMoved + rbTicksMoved) / 4.0;

            // Check if the average distance has reached the target.
            if (Math.abs(averageTicksMoved) >= Math.abs(ticks)) {
                holdHeading(200); // Stop the robot
                break;
            } else {
                double error = ticks - averageTicksMoved;
                double strafePower = error * DriveConstants.forwardP + Math.copySign(DriveConstants.forwardStallSpeed, error); // Re-using forwardP, consider a dedicated strafeP
                strafePower = Math.max(-0.5, Math.min(0.5, strafePower));
                // Call the main drive method with only the strafe component.
                drive(0, 0, strafePower);
            }
        }
    }

    public void strafeByTicks(int ticks, double power) {
        // Record the starting tick count for each motor.
        int[] startTicks = {
                leftFrontMotor.getCurrentPosition(),
                rightFrontMotor.getCurrentPosition(),leftBackMotor.getCurrentPosition(),
                rightBackMotor.getCurrentPosition()
        };

        // A small constant power to overcome static friction.
        // The sign is positive for right strafe, negative for left.

        while (true) {
            // For strafing, some motors move forward and others backward.
            int lfTicksMoved = leftFrontMotor.getCurrentPosition() - startTicks[0];
            int rfTicksMoved = rightFrontMotor.getCurrentPosition() - startTicks[1];
            int lbTicksMoved = leftBackMotor.getCurrentPosition() - startTicks[2];
            int rbTicksMoved = rightBackMotor.getCurrentPosition() - startTicks[3];

            // lf: +ticks, rf: -ticks, lb: -ticks, rb: +ticks
            double averageTicksMoved = (lfTicksMoved - rfTicksMoved - lbTicksMoved + rbTicksMoved) / 4.0;

            // Check if the average distance has reached the target.
            if (Math.abs(averageTicksMoved) >= Math.abs(ticks)) {
                holdHeading(200); // Stop the robot
                break;
            } else {
                double error = ticks - averageTicksMoved;
                double strafePower = error * DriveConstants.forwardP + Math.copySign(DriveConstants.forwardStallSpeed, error); // Re-using forwardP, consider a dedicated strafeP
                strafePower = Math.max(-power, Math.min(power, strafePower));
                // Call the main drive method with only the strafe component.
                drive(0, 0, strafePower);
            }
        }
    }

    public void holdHeading(long milliseconds) {
        long endTime = System.currentTimeMillis() + milliseconds;
        while (System.currentTimeMillis() < endTime) {
            drive(0, 0, 0);
        }
    }
    // In DriveSubsystem.java

    public void rotateToAngle(double targetAngle) {
        // Ensure targetAngle is within a standard -180 to 180 range if needed,
        // although the math below handles it.

        // Smallest power to apply to overcome static friction and ensure the robot moves.

        while (true) {
            // 1. GET THE LATEST ANGLE READING INSIDE THE LOOP
            // The IMU returns angles in the range of -180 to 180 degrees.
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw();

            // 2. CALCULATE THE SHORTEST PATH TO THE TARGET
            // This math correctly handles "wrapping" around the -180/180 degree boundary.
            double error = targetAngle - currentAngle;
            if (error > 180) {
                error -= 360;
            } else if (error < -180) {
                error += 360;
            }

            // 3. CHECK IF WE HAVE REACHED THE TARGET
            // Exit the loop if we are within the acceptable tolerance.
            if (Math.abs(error) <= 1.0) {
                break; // Exit the loop
            }

            // 4. CALCULATE POWER USING A P-CONTROLLER
            // Proportional control slows the robot down as it nears the target.
            double rotatePower = error * DriveConstants.autoTurnP;

            // 5. ADD STALL SPEED AND CLAMP THE POWER
            // Ensure we always have enough power to move, but never exceed 1.0.
            if (Math.abs(rotatePower) < DriveConstants.rotationStallSpeed) {
                rotatePower = Math.copySign(DriveConstants.rotationStallSpeed, rotatePower);
            }
            rotatePower = Math.max(-1.0, Math.min(1.0, rotatePower)); // Clamp between -1.0 and 1.0

            // 6. CALL THE DRIVE FUNCTION
            drive(0, -rotatePower, 0);
        }

        // 7. STOP THE ROBOT AND HOLD THE FINAL POSITION
        // Use holdHeading to actively resist any drift after the turn is complete.
        holdHeading(200); // Hold for 200 milliseconds
    }

    public void rotateToAngle(double targetAngle, double power) {

        while (true) {
            // 1. GET THE LATEST ANGLE READING INSIDE THE LOOP
            // The IMU returns angles in the range of -180 to 180 degrees.
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw();

            // 2. CALCULATE THE SHORTEST PATH TO THE TARGET
            // This math correctly handles "wrapping" around the -180/180 degree boundary.
            double error = targetAngle - currentAngle;
            if (error > 180) {
                error -= 360;
            } else if (error < -180) {
                error += 360;
            }

            // 3. CHECK IF WE HAVE REACHED THE TARGET
            // Exit the loop if we are within the acceptable tolerance.
            if (Math.abs(error) <= 1.0) { // 1.0 degree tolerance
                break; // Exit the loop
            }

            // 4. CALCULATE POWER USING A P-CONTROLLER
            // Proportional control slows the robot down as it nears the target.
            double rotatePower = error * DriveConstants.autoTurnP;

            // 5. ADD STALL SPEED AND CLAMP THE POWER
            // Ensure we always have enough power to move, but never exceed 1.0.
            if (Math.abs(rotatePower) < DriveConstants.rotationStallSpeed) {
                rotatePower = Math.copySign(DriveConstants.rotationStallSpeed, rotatePower);
            }
            rotatePower = Math.max(-power, Math.min(power, rotatePower)); // Clamp between -1.0 and 1.0

            // 6. CALL THE DRIVE FUNCTION
            drive(0, -rotatePower, 0);
        }

        // 7. STOP THE ROBOT AND HOLD THE FINAL POSITION
        // Use holdHeading to actively resist any drift after the turn is complete.
        holdHeading(200); // Hold for 200 milliseconds
    }

}
