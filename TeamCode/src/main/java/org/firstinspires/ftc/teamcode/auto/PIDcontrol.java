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
    double Kp  = 0;
    double integralSum = 0;
    double Ki  = 0;
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
