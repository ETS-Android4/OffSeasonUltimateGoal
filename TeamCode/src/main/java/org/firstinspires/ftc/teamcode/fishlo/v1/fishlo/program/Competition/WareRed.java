package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TrackingWheelForwardOffsetTuner;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Intake;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.TSEDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Vector;

import kotlin.math.UMathKt;

@Autonomous
public class WareRed extends FishloAutonomousProgram {
    TSEDetectionPipeline.BarcodePosition position;
    ElapsedTime timer;
    SampleMecanumDrive mdrive;



    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        timer = new ElapsedTime();
        telemetry.setAutoClear(true);
        drive.initGyro();
        vision.initVision();
        while (!isStarted()) {
            position = vision.getPlacement();
            telemetry.addData("Position", position);
            telemetry.update();
        }
    }

    // strafe right is positive power, strafe left is negative power!
//measurements subject to change
    //Re-push cuz rahul is bad
    @Override
    public void main() {
        telemetry.addLine("I hate ftc");
        telemetry.update();
        vision.stop();
        telemetry.clear();
        mdrive = new SampleMecanumDrive(hardwareMap);
        //sleep(200);
        mdrive.setPoseEstimate(new Pose2d());
        Pose2d start_pose = new Pose2d(0, 0, Math.toRadians(0));

        switch (position) {
            case LEFT:
                intake.armToLevel(0, false, 0);
                break;
            case CENTER:
                intake.armToLevel(1, false, 0);
                break;
            case RIGHT:
                intake.armToLevel(2, false, 0);
                break;
            case NULL:
                int random = (int)(Math.random()*3);
                intake.armToLevel(random, false, 0);
                telemetry.addData("Default Postion", random);
                telemetry.update();
        }
        telemetry.addLine("Before trajectory");

        Trajectory to_hub  = mdrive.trajectoryBuilder(start_pose)
                .splineToConstantHeading(new Vector2d(19, 25), Math.toRadians(0))
                .build();
        telemetry.addLine("After");
        mdrive.followTrajectory(to_hub);
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(500);
        intake.intake(Intake.IntakeState.OFF);
        Trajectory go_back = mdrive.trajectoryBuilder(to_hub.end())
                .lineTo(new Vector2d(10,25))
                .build();
        mdrive.followTrajectory(go_back);
        intake.stop();
        intake.resetEncoder();
        telemetry.addLine("Before turn");
        telemetry.update();
        mdrive.turn(Math.toRadians(-78));
        telemetry.addLine("finished turning");
        telemetry.update();


        Pose2d lpose= new Pose2d(go_back.end().getX(), go_back.end().getY(), Math.toRadians(-90));
        Trajectory ware = mdrive.trajectoryBuilder(lpose)
                .splineToConstantHeading(new Vector2d(-15,10), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-15, -30), Math.toRadians(-90))
                .build();
        telemetry.addLine("i stg");
        telemetry.update();
        mdrive.followTrajectory(ware);
        telemetry.addLine("i stg part 2");
        telemetry.update();
        intake.intake(Intake.IntakeState.ON);
        Pose2d turn_pose = new Pose2d(ware.end().getX(), ware.end().getY(), Math.toRadians(-90));
        Trajectory Test = mdrive.trajectoryBuilder(turn_pose)
                .splineToConstantHeading(new Vector2d(-15, -30), Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    intake.intake(Intake.IntakeState.ON);

                })
                .splineToConstantHeading(new Vector2d(-15, -38), Math.toRadians(-90),
                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();

        telemetry.addLine("built the alt");
        telemetry.update();
        mdrive.followTrajectory(Test);

        telemetry.addLine("about to start new trajectory");
        telemetry.update();
        Trajectory block_place = mdrive.trajectoryBuilder(Test.end())
                .splineTo(new Vector2d(-15, 20), Math.toRadians(-90))
                .build();
        telemetry.addLine("Built Trajectory block_place");
        telemetry.update();
        mdrive.followTrajectory(block_place);
        telemetry.addLine("Finished running Trajectory block_place");
        telemetry.update();
        Trajectory strafe_left = mdrive.trajectoryBuilder(block_place.end())
                .strafeLeft(5)
                .build();
        telemetry.addLine("built strafing");
        telemetry.update();
        mdrive.followTrajectory(strafe_left);
        telemetry.addLine("Finished strafing");
        telemetry.update();
    }


}


