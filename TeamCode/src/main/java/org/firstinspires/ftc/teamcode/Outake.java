package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outake {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor feeder = null;
    private DcMotor outakeR = null;
    private DcMotor outakeL = null;

    public enum PowerState {
        RUN(1),
        NOTRUN(0),
        REVERSE(-1);
        private final double power;
        PowerState(double power) {
            this.power = power;
        }
    }

}
