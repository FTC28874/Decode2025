package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Intake Outake", group="Linear OpMode")
public class IntakeOutake {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intake = null;
    private DcMotor feeder = null;
    private DcMotor outakeR = null;
    private DcMotor outakeL = null;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        outakeR = hardwareMap.get(DcMotor.class, "outakeR");
        outakeL = hardwareMap.get(DcMotor.class, "outakeL");

        outakeL.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

    }

}
