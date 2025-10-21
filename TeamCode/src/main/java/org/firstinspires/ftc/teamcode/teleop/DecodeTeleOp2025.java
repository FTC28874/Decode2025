package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Outake;
import org.firstinspires.ftc.teamcode.robot.Intake;

@TeleOp(name="Decode TeleOp 2025", group="Linear OpMode")
public class DecodeTeleOp2025 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor driveFL = null;
    private DcMotor driveFR = null;
    private DcMotor driveBL = null;
    private DcMotor driveBR = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // ---- DRIVE CONTROL ----
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x + (gamepad2.right_stick_x / 2); //gives gamepad 2 precise power over yaw

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double powerFL  = axial + lateral + yaw;
            double powerFR = axial - lateral - yaw;
            double powerBL   = axial - lateral + yaw;
            double powerBR  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(powerFL), Math.abs(powerFL));
            max = Math.max(max, Math.abs(powerBL));
            max = Math.max(max, Math.abs(powerBR));

            if (max > 1.0) {
                powerFL  /= max;
                powerFR /= max;
                powerBL   /= max;
                powerBR  /= max;
            }

            // Send calculated power to wheels
            driveFL.setPower(powerFL);
            driveFR.setPower(powerFR);
            driveBL.setPower(powerBL);
            driveBR.setPower(powerBR);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", powerFL, powerFR);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", powerBL, powerBR);
            telemetry.update();

            // ---- SHOOTER / INTAKE CONTROL ----
            if (gamepad2.right_bumper) {
                Outake.RunShooter();
            } else if (!gamepad2.right_bumper) {
                Outake.StopShooter();
            }
            if (gamepad2.x) {
                Outake.RunFeeder();
            } else if (!gamepad2.x) {
                Outake.StopFeeder();
            }
            if (gamepad2.left_bumper) {
                Intake.runIntake();
            } else if (!gamepad2.left_bumper) {
                Intake.stopIntake();
            }
            if (gamepad2.a) {
                Intake.reverseIntake();
                Outake.ReverseFeeder();
            } else if (!gamepad2.a) {
                Intake.stopIntake();
                Outake.StopShooter();
            }

        }
    }}
