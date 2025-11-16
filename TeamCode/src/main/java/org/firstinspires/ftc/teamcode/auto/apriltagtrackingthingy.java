package org.firstinspires.ftc.teamcode.auto;
import org.firstinspires.ftc.teamcode.mechanisms.apriltag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class apriltagtrackingthingy extends OpMode{
    apriltag apriltag = new apriltag();

    @Override
    public void init() {
        apriltag.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        apriltag.update();
        AprilTagDetection id20 = apriltag.getTagBySpecificId(24);
        telemetry.addData("id20 string", id20.toString());
    }
}