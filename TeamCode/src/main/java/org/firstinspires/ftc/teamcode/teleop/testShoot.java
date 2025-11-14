package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Outake;
import org.firstinspires.ftc.teamcode.robot.Intake;
//asdlfhasdf
@TeleOp(name="TestShooter", group="Linear OpMode")
public class testShoot extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        while (opModeIsActive()) {

            double shooterPower = -gamepad2.right_stick_y;

            Outake.SetShooterPower(shooterPower);

            if (gamepad2.x) {
                Outake.RunFeeder();
            } else if (!gamepad2.x) {
                Outake.StopFeeder();
            }

        }

    }

}
