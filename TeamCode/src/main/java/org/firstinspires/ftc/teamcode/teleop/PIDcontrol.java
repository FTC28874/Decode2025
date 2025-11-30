package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PID Control Test", group="Linear OpMode")
@Disabled
public class PIDcontrol extends LinearOpMode {

    // --- UPDATED STARTING GAINS (Based on 6000 RPM motors) ---
    // These are still starting points and MUST be tuned on your robot!
    double Kp  = 0.001;     // Start small. Use to fix residual error.
    double Ki  = 0.0;       // Start at 0. Use to eliminate steady-state error.
    double Kd  = 0.0;       // Start at 0. Use to dampen overshoot/oscillations.
    double Kf  = 0.0004;    // Feedforward based on 1.0 / 2800 Max Ticks/sec

    // Internal Variables
    DcMotorEx motor;
    DcMotorEx motor1;
    double integralSum = 0;
    ElapsedTime timer  = new ElapsedTime();
    private double lastError = 0;

    // Set a realistic target velocity (e.g., 2000 ticks/sec, which is ~4300 RPM)
    double TARGET_VELOCITY = 500.0;

    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "shooterL");
        motor1 = hardwareMap.get(DcMotorEx.class, "shooterR");

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Assuming both motors are controlled by the same PID output
            double currentVelocity = motor.getVelocity();
            double power = PIDControl(TARGET_VELOCITY, currentVelocity);

            motor.setPower(power);
            motor1.setPower(power);

            if (gamepad1.dpad_up) {
                TARGET_VELOCITY = TARGET_VELOCITY + 10.0;
                while (gamepad1.dpad_up) {}
            } else if (gamepad1.dpad_down) {
                TARGET_VELOCITY = TARGET_VELOCITY - 10.0;
                while (gamepad1.dpad_down) {}
            }

            // Telemetry is crucial for tuning these values
            telemetry.addData("Target Velocity (ticks/s)", TARGET_VELOCITY);
            telemetry.addData("Actual Velocity (ticks/s)", currentVelocity);
            telemetry.addData("Calculated Power", power);
            telemetry.addData("Error", TARGET_VELOCITY - currentVelocity);
            telemetry.update();
        }
    }

    public double PIDControl(double reference, double state) {
        // 1. Calculate Error
        double error = reference - state;

        // Time since last loop iteration (Delta Time)
        double dt = timer.seconds();
        timer.reset();

        // 2. Integral Component (with anti-windup check)
        // Adjust '100.0' based on the magnitude of error you want to ignore
        if (Math.abs(error) < 100.0) {
            integralSum += error * dt;
        }

        // 3. Derivative Component
        double derivative = (error - lastError) / dt;
        lastError = error;

        // 4. Full PID + Feedforward Calculation
        double output = (error * Kp) +
                (integralSum * Ki) +
                (derivative * Kd) +
                (reference * Kf);

        // 5. Clamp Output Power
        return Range.clip(output, -1.0, 1.0);
    }
}