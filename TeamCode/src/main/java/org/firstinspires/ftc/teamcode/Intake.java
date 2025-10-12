package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intake = null;
    private DcMotor feeder = null;

    public enum PowerState {
        RUN(1.0),
        NOTRUN(0.0),
        REVERSE(-1.0);
        private final double power;
        PowerState(double power) {
            this.power = power;
        }
    }

    public void runIntake() {
        intake.setPower(RUN);
    }
    public void stopIntake() {
        intake.setPower(0);
    }
    public void reverseIntake() {
        intake.setPower(-1);
    }
    public void runFeeder() {
        feeder.setPower(1);
    }



}
