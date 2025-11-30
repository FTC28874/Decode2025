package org.firstinspires.ftc.teamcode.robot;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Outake utility class. Designed for simple static-style access from OpModes.
 * Call Outake.init(hardwareMap) once during OpMode init before using other methods.
 */
public class Outake {
    private static ElapsedTime runtime = new ElapsedTime();
    private static DcMotor feeder = null;
    private static DcMotor shooterR = null;
    private static DcMotor shooterL = null;
    DcMotorEx motor;
    DcMotorEx motor1;
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




    /**
     * Initialize Outake hardware. Must be called once before using static methods.
     */
    public static void init(HardwareMap hardwareMap) {
        shooterR = hardwareMap.get(DcMotor.class, "shooterR");
        shooterL = hardwareMap.get(DcMotor.class, "shooterL");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        // Set directions - adjust if motors spin the wrong way
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public enum PowerState {
        RUN(1.0),
        NOT_RUN(0.0),
        SHOOTER_RUN(0.5),
        REVERSE(-1.0);
        private final double power;
        PowerState(double power) {
            this.power = power;
        }
    }


    public static void RunShooter() {
        if (shooterL != null && shooterR != null) {
            shooterL.setPower(PowerState.SHOOTER_RUN.power);
            shooterR.setPower(PowerState.SHOOTER_RUN.power);
        }
    }

    public static void SetShooterPower(double shooterPower) {
        if (shooterL != null && shooterR != null) {
            shooterL.setPower(shooterPower);
            shooterR.setPower(shooterPower);
        }
    }

    public static void StopShooter() {
        if (shooterL != null && shooterR != null) {
            shooterL.setPower(PowerState.NOT_RUN.power);
            shooterR.setPower(PowerState.NOT_RUN.power);
        }
    }

    public static void RunFeeder() {
        if (feeder != null) feeder.setPower(PowerState.RUN.power);
    }

    public static void StopFeeder() {
        if (feeder != null) feeder.setPower(PowerState.NOT_RUN.power);
    }

    public static void ReverseFeeder() {
        if (feeder != null) feeder.setPower(PowerState.REVERSE.power);
    }

}
