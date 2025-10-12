package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outake {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor feeder = null;
    private DcMotor outakeR = null;
    private DcMotor outakeL = null;

    public Outake() {
        outakeR = hardwareMap.get(DcMotor.class, "outakeR");
        outakeL = hardwareMap.get(DcMotor.class, "outakeL");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        outakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        outakeR.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public enum PowerState {
        RUN(1),
        NOTRUN(0),
        REVERSE(-1);
        private final double power;
        PowerState(double power) {
            this.power = power;
        }
    }

    public void RunShooter() {
        outakeL.setPower(PowerState.RUN.power);
        outakeR.setPower(PowerState.RUN.power);
        feeder.setPower(PowerState.RUN.power);
    }
    public void StopShooter() {
        outakeL.setPower(PowerState.NOTRUN.power);
        outakeR.setPower(PowerState.NOTRUN.power);
        feeder.setPower(PowerState.NOTRUN.power);
    }
    public void reverseFeeder() {
        feeder.setPower(PowerState.REVERSE.power);
    }


}
