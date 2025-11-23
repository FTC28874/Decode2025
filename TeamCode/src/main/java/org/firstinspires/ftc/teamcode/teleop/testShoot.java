package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Outake;
//asdlfhasdf
@TeleOp(name="TestShooter", group="Linear OpMode")
public class testShoot extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        org.firstinspires.ftc.teamcode.robot.Outake.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            double shooterPower = 0.5;

            Outake.SetShooterPower(shooterPower);

        }

    }

}
