package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous
@Disabled
public class apriltagtrackingthingy extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);
        aprilTagWebcam.displayDetectionTelemetry(id24);
        telemetry.addData("id24 string", id24.toString());
    }
}