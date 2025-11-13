package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "My Autonomous Version", group = "Autonomous")
public class myversion extends LinearOpMode {

    // Drivetrain motors
    private DcMotor driveFL, driveFR, driveBL, driveBR;

    // IMU
    private BNO055IMU imu;

    // Constants — tune these for your robot
    private static final double TICKS_PER_REV = 537.6; // For a REV Core Hex or similar motor
    private static final double WHEEL_DIAMETER_IN = 3.779; // inches
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);
    
    // Proportional gain for steering correction when driving straight.
    // Tune this value until the robot drives straight without oscillating.
    private static final double DRIVE_KP = 0.03;

    @Override
    public void runOpMode() {
        // Map hardware
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        // Motor direction (set to match your build)
        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

        // Use encoders
        for (DcMotor m : new DcMotor[]{driveFL, driveFR, driveBL, driveBR}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Init IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Get the robot's starting heading
            double startHeading = getHeading();
            
            // Example autonomous sequence
            driveStraight(24, 0.5, startHeading);   // drive forward 24 inches, maintaining original heading
            turnToAngle(90, 0.4);                     // turn 90 degrees right
            driveStraight(12, 0.5, 90);             // drive forward 12 inches, maintaining 90-degree heading
        }
    }

    // ===== Helper Methods =====

    /**
     * Drives straight for a set number of inches while using the IMU to hold a heading.
     * @param inches The distance to drive.
     * @param power The base power to apply.
     * @param targetHeading The heading (in degrees) to maintain.
     */
    private void driveStraight(double inches, double power, double targetHeading) {
        int targetTicks = (int) (inches * TICKS_PER_INCH);

        // Reset encoders and set target
        for (DcMotor m : new DcMotor[]{driveFL, driveFR, driveBL, driveBR}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(targetTicks);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // This loop will run while the motors are busy, applying power and correcting heading.
        while (opModeIsActive() && (driveFL.isBusy() && driveFR.isBusy())) {
            double currentHeading = getHeading();
            double headingError = targetHeading - currentHeading;

            // Calculate steering correction
            double turnCorrection = headingError * DRIVE_KP;

            // Apply power to motors with steering correction
            driveFL.setPower(power - turnCorrection);
            driveBL.setPower(power - turnCorrection);
            driveFR.setPower(power + turnCorrection);
            driveBR.setPower(power + turnCorrection);

            telemetry.addData("Driving Straight", "Target: %d", targetTicks);
            telemetry.addData("Current Pos", "FL:%d FR:%d", driveFL.getCurrentPosition(), driveFR.getCurrentPosition());
            telemetry.addData("Heading", "Target: %.1f, Current: %.1f", targetHeading, currentHeading);
            telemetry.update();
        }

        stopDrive();
        
        // Set motors back to normal run mode
        for (DcMotor m : new DcMotor[]{driveFL, driveFR, driveBL, driveBR}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void turnToAngle(double targetAngle, double power) {
        double currentAngle = getHeading();
        double error = targetAngle - currentAngle;

        while (opModeIsActive() && Math.abs(error) > 2) { // tolerance of 2 degrees
            // Power should be proportional to the error, and slow down as it approaches.
            double turnPower = error * 0.015; // A simple proportional gain, tune this!
            // Ensure minimum power to overcome friction, but respect the sign of the error.
            turnPower = Math.copySign(Math.max(Math.abs(turnPower), 0.15), turnPower);
            // Cap the power at the requested max power
            turnPower = Math.copySign(Math.min(Math.abs(turnPower), power), turnPower);

            driveFL.setPower(turnPower);
            driveBL.setPower(turnPower);
            driveFR.setPower(-turnPower);
            driveBR.setPower(-turnPower);

            currentAngle = getHeading();
            error = targetAngle - currentAngle;

            telemetry.addData("Turning", "Target: %.1f, Current: %.1f", targetAngle, currentAngle);
            telemetry.update();
        }
        stopDrive();
    }

    private double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    private void stopDrive() {
        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);
    }
}
