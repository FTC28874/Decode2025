package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class PIDcontrol extends LinearOpMode {
    DcMotorEx motor;
    DcMotorEx motor1;
    //Kp (Proportional Gain): This value determines the immediate corrective action based on the current error (the difference between the desired state and the actual state). A higher Kp value results in a more aggressive, faster response, but if set too high, it can cause the system to overshoot the target and oscillate.
    double Kp  = 0;
    //Integral Sum: This variable in the code (e.g., integral_sum in the image) is the running total of all previous errors, typically updated in every loop iteration. It is multiplied by Ki to form the integral component of the overall control output.
    double integralSum = 0;
    //Ki (Integral Gain): This value is multiplied by the integral sum to account for the accumulation of past errors over time. The primary purpose of the integral term is to eliminate any persistent, steady-state error that the proportional term might not fix on its own. The integral sum builds up until the error is eliminated, even if a non-zero output is needed to hold the target position or speed.
    double Ki  = 0;
    //Kd (Derivative Gain): This value determines the response based on the rate of change of the error (how fast the system is moving toward or away from the target). The derivative term helps to dampen oscillations and improve stability, allowing the system to react quickly to changes without overshooting the setpoint.
    double Kd = 0;
    double Kf = 0;
    ElapsedTime timer  = new ElapsedTime();
    private double lastError = 0;
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "shootermotorL");
        motor1 = hardwareMap.get(DcMotorEx.class, "shootermotorR");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
        double power = PIDControl(100, motor.getVelocity());
        motor.setPower(power);
        motor1.setPower(power);
        }
    }
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();
        return (error * Kp) + (derivative * Kd) + (integralSum + Ki) + (reference * Kf);
    }
}
