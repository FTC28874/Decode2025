package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class autopathing extends OpMode {
private Follower follower;
private Timer pathTimer, opModeTimer;
public enum  PathState{
    //START POSITION_END POSITION
    //DRIVE > MOVEMENT STATE
    //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
    DRIVE_STARTPOS_INTAKE_POS,
    READY_INTAKE,
    DRIVE_TO_ARTIFACT


}
PathState pathState;

private final Pose startPose = new Pose(20.386, 122.4, Math.toRadians(138));
private final Pose intakePose = new Pose(54.42935779816514, 84.02201834862385, Math.toRadians(180));
private final Pose artifactPose = new Pose(17.702752293577984, 84.02201834862385, Math.toRadians(180));

private PathChain  driveStartPosIntakePos, driveIntakePosArtifactPos;
public void buildPaths(){
    //include coordinates for starting pose > ending pose
    driveStartPosIntakePos = follower.pathBuilder()
            .addPath(new BezierLine(startPose, intakePose))
            .setLinearHeadingInterpolation(startPose.getHeading(), intakePose.getHeading())
            .build();
    driveIntakePosArtifactPos = follower.pathBuilder()
            .addPath(new BezierLine(intakePose, artifactPose))
            .setLinearHeadingInterpolation(intakePose.getHeading(), artifactPose.getHeading())
            .build();
    }
    public  void statePathUpdate(){
    switch (pathState) {
        case DRIVE_STARTPOS_INTAKE_POS:
            follower.followPath(driveStartPosIntakePos, true);
            setPathState(PathState.READY_INTAKE); //reset timer
            break;
        case READY_INTAKE:
            // TODO add logic to flywheel shooter
            //check if follower has done path
            if (!follower.isBusy() && pathTimer.getElapsedTime() > 5) {
                follower.followPath(driveIntakePosArtifactPos, true);
                setPathState(PathState.DRIVE_TO_ARTIFACT);

                //next state here...
            }
            break;
        case DRIVE_TO_ARTIFACT:
            if (!follower.isBusy()) {
                telemetry.addLine("Done all given paths");
            }
        default:
            telemetry.addLine("No more states...");
            break;

    }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_INTAKE_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower((hardwareMap));
        // TODO add any other init mechanisms
        buildPaths();
        follower.setPose(startPose);

    }
    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());
    }
}
