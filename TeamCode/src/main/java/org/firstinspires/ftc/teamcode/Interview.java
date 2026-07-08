package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.ShooterSystem;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.Component;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
//84 84, 128 84


public class Interview extends LinearOpMode {


    private int pathState;
    private Timer pathTimer, opmodeTimer;

    private final Pose MoveToShootingPos = new Pose(84, 84, Math.toRadians(0));
    private final Pose PickUpBalls = new Pose(128, 84, Math.toRadians(0));

    private final Pose StartPos = new Pose(132,132, Math.toRadians(0));
    private PathChain  MoveToShootingPosition,PickUpBallsOnSpike,SecondShootingPos;

//    public class ShooterSystem implements Subsystem {
//        public ShooterSystem INSTANCE = new ShooterSystem();
//
//        private MotorEx Shooter = new MotorEx("Shooter");
//
//        public Command stopMotor = instant(() -> {
//            Shooter.setPower(0.0);
//
//        }); // automatically requires this
//    }





    @Override
    public void runOpMode () {


        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        Paths();
        follower.setStartingPose(StartPos);


        waitForStart();

        opmodeTimer.resetTimer();
        setPathState(0);

        while (opModeIsActive()){
            follower.update();
            AutoUpdate();
            if(pathState == 0){
            }

        }




    }

    public void Paths() {
        MoveToShootingPosition = follower.pathBuilder()
                .addPath(new BezierLine(StartPos, MoveToShootingPos))
                .setLinearHeadingInterpolation(StartPos.getHeading(), MoveToShootingPos.getHeading())
                .build();
        PickUpBallsOnSpike = follower.pathBuilder()
                .addPath(new BezierLine(MoveToShootingPos, PickUpBalls))
                .setLinearHeadingInterpolation(MoveToShootingPos.getHeading(), PickUpBalls.getHeading())
                .build();
        SecondShootingPos = follower.pathBuilder()
                .addPath(new BezierLine(PickUpBalls, MoveToShootingPos))
                .setLinearHeadingInterpolation(PickUpBalls.getHeading(), MoveToShootingPos.getHeading())
                .build();
    }
    public void AutoUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(MoveToShootingPosition);

                setPathState(1);

                break;
            case 1:
                if (!follower.isBusy()) {

                    follower.followPath(PickUpBallsOnSpike, true);
                    setPathState(2);
                }
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(SecondShootingPos, true);
                    setPathState(3);
                };
            case 3:
                if(!follower.isBusy()){
                    setPathState(-1);
                }
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }










}

