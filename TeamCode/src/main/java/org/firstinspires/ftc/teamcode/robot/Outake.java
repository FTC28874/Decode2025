package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outake {
    private ElapsedTime runtime = new ElapsedTime();
    private static DcMotor feeder = null;
    private static DcMotor outakeR = null;
    private static DcMotor outakeL = null;

    public Outake() {
        outakeR = hardwareMap.get(DcMotor.class, "outakeR");
        outakeL = hardwareMap.get(DcMotor.class, "outakeL");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        outakeL.setDirection(DcMotorSimple.Direction.REVERSE); // may have to tweak these lines 19 and 20
        outakeR.setDirection(DcMotorSimple.Direction.FORWARD); // in case that they are spinning in wrong direction
    }
    public enum PowerState {
        RUN(1),
        NOT_RUN(0),
        REVERSE(-1);
        private final double power;
        PowerState(double power) {
            this.power = power;
        }
    }

    public static void RunShooter() {
        outakeL.setPower(PowerState.RUN.power);
        outakeR.setPower(PowerState.RUN.power);
    }
    public static void StopShooter() {
        outakeL.setPower(PowerState.NOT_RUN.power);
        outakeR.setPower(PowerState.NOT_RUN.power);
    }
    public static void RunFeeder() {
        feeder.setPower(PowerState.RUN.power);
    }
    public static void StopFeeder() {
        feeder.setPower(PowerState.NOT_RUN.power);
    }
    public static void ReverseFeeder() {
        feeder.setPower(PowerState.REVERSE.power);}


}
