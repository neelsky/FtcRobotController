package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "GoingInASquare", group = "Interview")
public class InterviewTest extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose corner1   = new Pose(24, 0, Math.toRadians(0));
    private final Pose corner2   = new Pose(24, 24, Math.toRadians(0));
    private final Pose corner3   = new Pose(0, 24, Math.toRadians(0));

    private PathChain side1, side2, side3, side4;

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();

        opmodeTimer.resetTimer();
        setPathState(0);

        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    public void buildPaths() {
        side1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, corner1))
                .setLinearHeadingInterpolation(startPose.getHeading(), corner1.getHeading())
                .build();

        side2 = follower.pathBuilder()
                .addPath(new BezierLine(corner1, corner2))
                .setLinearHeadingInterpolation(corner1.getHeading(), corner2.getHeading())
                .build();

        side3 = follower.pathBuilder()
                .addPath(new BezierLine(corner2, corner3))
                .setLinearHeadingInterpolation(corner2.getHeading(), corner3.getHeading())
                .build();

        side4 = follower.pathBuilder()
                .addPath(new BezierLine(corner3, startPose))
                .setLinearHeadingInterpolation(corner3.getHeading(), startPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(side1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(side2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(side3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(side4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}