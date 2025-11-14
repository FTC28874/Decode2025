package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
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
        REVERSE(-1.0);
        private final double power;
        PowerState(double power) {
            this.power = power;
        }
    }


    public static void RunShooter() {
        if (shooterL != null && shooterR != null) {
            shooterL.setPower(PowerState.RUN.power);
            shooterR.setPower(PowerState.RUN.power);
        }
    }

    public static void SetShooterPower(double power) {
        shooterL.setPower(power);
        shooterR.setPower(power);
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
