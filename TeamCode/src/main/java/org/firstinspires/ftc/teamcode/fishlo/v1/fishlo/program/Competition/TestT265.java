package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.T265MecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class TestT265 extends FishloAutonomousProgram {

    public static T265MecanumDrive drive;

    private double X = 30;
    private double Y = 30;


    @Override
    protected Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    @Override
    public void preMain() {
        drive = new T265MecanumDrive(this);
    }

    @Override
    public void main() {
        if(isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Vector2d(X, Y), 0)
                        .build()
        );

        telemetry.addLine("Complete!");
    }
}
