package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//asdf
@Autonomous(name="MecanumAuto", group="LinearOpmode")
@Disabled
public class autonomous2 extends LinearOpMode {

    // Drivetrain motors
    private DcMotor driveFL, driveFR, driveBL, driveBR;

    // Constants for robot geometry and motors - TUNE THESE FOR YOUR ROBOT
    private static final double TICKS_PER_REV = 537.6; // Example for a REV Core Hex Motor
    private static final double WHEEL_DIAMETER_IN = 4.09449; // inches
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_IN * Math.PI);

    // Turning can be tricky with only encoders. This is an approximation.
    // Represents the inches the wheels travel to turn the robot.
    private static final double INCHES_PER_DEGREE = 0.2; // TUNE THIS VALUE

    @Override
    public void runOpMode() {
        // Initialize motors
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        // Reverse right side motors for correct mecanum drive
        driveFR.setDirection(DcMotor.Direction.REVERSE);
        driveBR.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to stop when power is zero
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (opModeIsActive()) {
            // Example autonomous sequence using INCHES and DEGREES:
            drive(48, 0.5);   // drive forward 48 inches
            strafe(-12, 0.5);  // strafe left 12 inches (positive is right)
            turn(90, 0.4);      // turn right 90 degrees
        }
    }

    // --- High-Level Movement Methods ---

    private void drive(double inches, double power) {
        int ticks = (int) (inches * TICKS_PER_INCH);
        setRelativeTargetPosition(ticks, ticks, ticks, ticks);
        runToPosition(power);
    }

    private void strafe(double inches, double power) {
        int ticks = (int) (inches * TICKS_PER_INCH);
        // Mecanum strafing pattern (front-left and back-right are positive)
        setRelativeTargetPosition(ticks, -ticks, -ticks, ticks);
        runToPosition(power);
    }

    private void turn(double degrees, double power) {
        int ticks = (int) (degrees * INCHES_PER_DEGREE * TICKS_PER_INCH);
        // Left wheels forward, right wheels backward to turn right
        setRelativeTargetPosition(ticks, -ticks, ticks, -ticks);
        runToPosition(power);
    }

    // --- Low-Level Helper Methods ---

    private void setRelativeTargetPosition(int fl, int fr, int bl, int br) {
        driveFL.setTargetPosition(driveFL.getCurrentPosition() + fl);
        driveFR.setTargetPosition(driveFR.getCurrentPosition() + fr);
        driveBL.setTargetPosition(driveBL.getCurrentPosition() + bl);
        driveBR.setTargetPosition(driveBR.getCurrentPosition() + br);
    }

    private void runToPosition(double power) {
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);

        while (opModeIsActive() && (driveFL.isBusy() || driveFR.isBusy() || driveBL.isBusy() || driveBR.isBusy())) {
            telemetry.addData("Status", "Running to target");
            telemetry.addData("Target", "FL:%d, FR:%d, BL:%d, BR:%d",
                    driveFL.getTargetPosition(), driveFR.getTargetPosition(), driveBL.getTargetPosition(), driveBR.getTargetPosition());
            telemetry.addData("Current", "FL:%d, FR:%d, BL:%d, BR:%d",
                    driveFL.getCurrentPosition(), driveFR.getCurrentPosition(), driveBL.getCurrentPosition(), driveBR.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
        // Set mode back to default for the next command or teleop
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopMotors() {
        setPower(0);
    }

    private void setPower(double power) {
        driveFL.setPower(power);
        driveFR.setPower(power);
        driveBL.setPower(power);
        driveBR.setPower(power);
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        driveFL.setMode(mode);
        driveFR.setMode(mode);
        driveBL.setMode(mode);
        driveBR.setMode(mode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        driveFL.setZeroPowerBehavior(behavior);
        driveFR.setZeroPowerBehavior(behavior);
        driveBL.setZeroPowerBehavior(behavior);
        driveBR.setZeroPowerBehavior(behavior);
    }
}
