package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intake = null;
    private DcMotor feeder = null;

    public Intake() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
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

    public void runIntake() {
        intake.setPower(PowerState.RUN.power);
    }
    public void stopIntake() {
        intake.setPower(PowerState.NOT_RUN.power);
    }
    public void reverseIntake() {
        intake.setPower(PowerState.REVERSE.power);
    }


}
