package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Simple RI5W Auto", group="LinearOpMode")
public class SimpleRI5WAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor driveFL = null;
    private DcMotor driveBL = null;
    private DcMotor driveFR = null;
    private DcMotor driveBR = null;

    @Override
    public void runOpMode() {
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

            sleep(25000);

            driveBL.setPower(1);
            driveBR.setPower(1);
            driveFR.setPower(1);
            driveFL.setPower(1);

            sleep(200);

            driveBL.setPower(0);
            driveBR.setPower(0);
            driveFR.setPower(0);
            driveFL.setPower(0);
    }


}
