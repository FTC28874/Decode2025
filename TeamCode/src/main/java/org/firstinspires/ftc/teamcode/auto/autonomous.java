package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto_Encoder_IMU_Template", group = "Autonomous") class Auto_Encoder_IMU_Template  extends LinearOpMode {

    // Drivetrain motors (rename to match your config)
    private DcMotor driveFL, driveFR, driveBL, driveBR;
    private DcMotor getDriveFL, getDriveFR, getDriveBL, getDriveBR; // Hi There hiThere

    // IMU
    private BNO055IMU imu;

    // Constants — tune these for your robot
    private static final double TICKS_PER_INCH = 1120.0; //

    @Override
    public void runOpMode() {
        // Map hardware
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        // Motor direction (set to match your build)
        driveFL.setDirection(DcMotor.Direction.FORWARD);
        driveBL.setDirection(DcMotor.Direction.FORWARD);
        driveFR.setDirection(DcMotor.Direction.REVERSE);
        driveBR.setDirection(DcMotor.Direction.REVERSE);

        // Use encoders
        for (DcMotor m : new DcMotor[]{driveFL, driveFR, driveBL, driveBR}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Init IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);
//6767676676767676767676767676767676767676767. kinda reAL FOR That 676767676767-
    }
}