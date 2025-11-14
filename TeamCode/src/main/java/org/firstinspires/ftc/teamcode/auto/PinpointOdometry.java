package org.firstinspires.ftc.teamcode.auto;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class PinpointOdometry {

    public void runOpmMode() {

    }
//    // Motors
//    private DcMotor frontLeft, frontRight, backLeft, backRight;
//    private GoBildaPinpointDriver pinpoint;
//
//    // PID coefficients
//    private double kP_pos = 0.05;
//    private double kI_pos = 0.0;
//    private double kD_pos = 0.003;
//
//    private double kP_heading = 0.03;
//    private double kI_heading = 0.0;
//    private double kD_heading = 0.002;
//
//    // Target position (in inches)
//    private double targetX = 24.0;
//    private double targetY = 24.0;
//    private double targetHeadingDeg = 0.0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        // Motor mapping
//        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
//        backRight  = hardwareMap.get(DcMotor.class, "backRight");
//
//        // Mecanum setup: reverse right side
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.REVERSE);
//
//        // Optional: brake when power = 0
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Odometry
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        pinpoint.setOffsets(0, 0,);
//        pinpoint.setEncoderResolution(8192.0);
//        pinpoint.resetPosAndIMU();
//
//        telemetry.addLine("Ready to start PID drive...");
//        telemetry.update();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        // Move to the point with a 1.5 inch tolerance and a 5 second timeout.
//        moveToPoint(targetX, targetY, targetHeadingDeg, 1.5, 5.0);
//        stopMotors();
//    }
//
//    private void moveToPoint(double targetX, double targetY, double targetHeadingDeg, double toleranceInches, double timeoutSeconds) {
//        double integralX = 0, integralY = 0, integralHeading = 0;
//        double lastErrorX = 0, lastErrorY = 0, lastErrorHeading = 0;
//        ElapsedTime pidTimer = new ElapsedTime();
//        ElapsedTime timeoutTimer = new ElapsedTime();
//
//        while (opModeIsActive() && timeoutTimer.seconds() < timeoutSeconds) {
//            pinpoint.update();
//            double x = pinpoint.getPosXInches();
//            double y = pinpoint.getPosYInches();
//            double headingRad = pinpoint.getHeadingRadians();
//            double headingDeg = Math.toDegrees(headingRad);
//
//            double errorX = targetX - x;
//            double errorY = targetY - y;
//
//            double distance = Math.hypot(errorX, errorY);
//            if (distance < toleranceInches) {
//                telemetry.addLine("Reached Target");
//                break; // close enough
//            }
//
//            double dt = pidTimer.seconds();
//            pidTimer.reset();
//
//            // --- PID for position ---
//            integralX += errorX * dt;
//            integralY += errorY * dt;
//
//            double derivativeX = 0, derivativeY = 0;
//            if (dt > 0) { // check for dt > 0 to prevent division by zero
//                derivativeX = (errorX - lastErrorX) / dt;
//                derivativeY = (errorY - lastErrorY) / dt;
//            }
//
//            double outputX = kP_pos * errorX + kI_pos * integralX + kD_pos * derivativeX;
//            double outputY = kP_pos * errorY + kI_pos * integralY + kD_pos * derivativeY;
//
//            lastErrorX = errorX;
//            lastErrorY = errorY;
//
//            // --- PID for heading ---
//            double errorHeading = angleWrap(targetHeadingDeg - headingDeg);
//            integralHeading += errorHeading * dt;
//            double derivativeHeading = 0;
//            if (dt > 0) { // check for dt > 0 to prevent division by zero
//                 derivativeHeading = (errorHeading - lastErrorHeading) / dt;
//            }
//            double turn = kP_heading * errorHeading + kI_heading * integralHeading + kD_heading * derivativeHeading;
//            lastErrorHeading = errorHeading;
//
//            // --- Convert field-centric to robot-centric ---
//            double cosA = Math.cos(headingRad);
//            double sinA = Math.sin(headingRad);
//            double robotX = outputX * cosA + outputY * sinA;
//            double robotY = -outputX * sinA + outputY * cosA;
//
//            // --- Mecanum drive equations ---
//            double fl = robotY + robotX + turn;
//            double fr = robotY - robotX - turn;
//            double bl = robotY - robotX + turn;
//            double br = robotY + robotX - turn;
//
//            // Normalize
//            double max = Math.max(1.0, Math.max(Math.abs(fl),
//                    Math.max(Math.abs(fr),
//                            Math.max(Math.abs(bl), Math.abs(br)))));
//            fl /= max; fr /= max; bl /= max; br /= max;
//
//            frontLeft.setPower(fl);
//            frontRight.setPower(fr);
//            backLeft.setPower(bl);
//            backRight.setPower(br);
//
//            telemetry.addData("Target", "(%.1f, %.1f)", targetX, targetY);
//            telemetry.addData("Current", "(%.1f, %.1f)", x, y);
//            telemetry.addData("Error (in)", "(%.2f, %.2f)", errorX, errorY);
//            telemetry.addData("Heading", "%.1f°", headingDeg);
//            telemetry.addData("Turn PID", "%.2f", turn);
//            telemetry.update();
//        }
//        if(timeoutTimer.seconds() >= timeoutSeconds){
//            telemetry.addLine("MoveToPoint Timed Out");
//        }
//    }
//
//    private void stopMotors() {
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//    }
//
//    /** Normalize heading error to [-180, 180] */
//    private double angleWrap(double angle) {
//        while (angle > 180) angle -= 360;
//        while (angle <= -180) angle += 360;
//        return angle;
//    }
}
