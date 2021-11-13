 package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

<<<<<<< HEAD
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
=======
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.T265MecanumDrive;
>>>>>>> 38dc0cfd9297e2c84fe2a488cfec77976a89c00f
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Drive extends SubSystem {

<<<<<<< HEAD
    private Motor frontLeft, frontRight, backLeft, backRight;

    HDrive xDrive;

    SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(robot.hardwareMap);

    Servo claw;
=======
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    T265MecanumDrive T265Drive = new T265MecanumDrive(robot.hardwareMap);
>>>>>>> 38dc0cfd9297e2c84fe2a488cfec77976a89c00f

    private enum DriveControls {
        TANK,
        ARCADE
    }

    DriveControls[] driveControls = {DriveControls.TANK, DriveControls.ARCADE};
    DriveControls driveType;
    int driveIndex = 0;
    /**
     * Construct a subsystem with the robot it applies to.
     *
     * @param robot
     */
    public Drive(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        frontLeft = new Motor(robot.hardwareMap, "frontLeft");
        frontRight = new Motor(robot.hardwareMap, "frontRight");
        backLeft = new Motor(robot.hardwareMap, "backLeft");
        backRight = new Motor(robot.hardwareMap, "backRight");
        claw = robot.hardwareMap.servo.get("claw");
        xDrive = new HDrive(frontLeft, frontRight, backLeft, backRight);
    }

    @Override
    public void handle() {
<<<<<<< HEAD
        double driveSpeed = robot.gamepad1.left_stick_y / 4;
        double turnSpeed = 0;

        turnSpeed = robot.gamepad1.right_stick_x / 4;

        driveIndex = 1;

        driveType = driveControls[driveIndex];
        if (robot.gamepad1.y) {
            claw.setPosition(0.5);
        }
        if (robot.gamepad1.a) {
            claw.setPosition(0);
        }
        mecanumDrive.setWeightedDrivePower(new Pose2d(
                robot.gamepad1.left_stick_y,
                -robot.gamepad1.left_stick_x,
                -robot.gamepad1.right_stick_x
        ));

=======
        double driveSpeed = -robot.gamepad1.left_stick_y;
        double rightY = robot.gamepad1.right_stick_y;
        double turnSpeed = 0;
        double strafeSpeed = robot.gamepad1.left_stick_x;


        if (Math.abs(robot.gamepad1.right_stick_x) > 0.1) {
            turnSpeed = robot.gamepad1.right_stick_x;
        }

        if(robot.gamepad1.x) {
            driveIndex = 0;
        }
        if(robot.gamepad1.b) {
            driveIndex = 1;
        }

        driveType = driveControls[driveIndex];
        runDrive(driveType, driveSpeed, strafeSpeed, turnSpeed, rightY, -driveSpeed);

>>>>>>> 38dc0cfd9297e2c84fe2a488cfec77976a89c00f
        robot.telemetry.addData("Drive - Dat - Drive Controls", driveType.name());
        robot.telemetry.addData("Drive - Dat - Drive Speed", driveSpeed);
        robot.telemetry.addData("Drive - Dat - Turn Speed", turnSpeed);
        robot.telemetry.addData("Drive - Dat - GamepadX", robot.gamepad1.left_stick_x);
<<<<<<< HEAD
=======
        robot.telemetry.addData("Drive - Dat - Strafe Speed", strafeSpeed);
        robot.telemetry.addData("Drive - Set - frontLeft", frontLeft.getPower());
        robot.telemetry.addData("Drive - Set - backLeft", backLeft.getPower());
        robot.telemetry.addData("Drive - Set - frontRight", frontRight.getPower());
        robot.telemetry.addData("Drive - Set - backRight", backRight.getPower());
>>>>>>> 38dc0cfd9297e2c84fe2a488cfec77976a89c00f
    }

    @Override
    public void stop() {
        
    }

    public void runDrive(DriveControls driveType, double driveSpeed, double strafeSpeed, double turnSpeed, double rightY, double leftY) {
        switch (driveType) {
            case ARCADE:
                T265Drive.setWeightedDrivePower(
                        new Pose2d(
                                driveSpeed,
                                -strafeSpeed,
                                -turnSpeed
                        )
                );
                break;

            case TANK:
                if (robot.gamepad1.right_bumper) {
                    if (robot.gamepad1.right_trigger < 0.5) {
                        strafe(0.5);
                    }
                    else if (robot.gamepad1.left_trigger < 0.5) {
                        strafe(0.3);
                    }
                    else {
                        strafe(1);
                    }
                }
                else if (robot.gamepad1.left_bumper) {
                    if (robot.gamepad1.right_trigger < 0.5) {
                        strafe(-0.5);
                    }
                    else if (robot.gamepad1.left_trigger < 0.5) {
                        strafe(-0.3);
                    }
                    else {
                        strafe(-1);
                    }
                }
                else {
                    left(-leftY);
                    right(-rightY);
                }
                break;

            default:
                driveType = DriveControls.TANK;
        }
    }

<<<<<<< HEAD
//    public void runDrive(DriveControls driveType) {
//        switch (driveType) {
//            case ARCADE:
//                xDrive.driveRobotCentric(
//                        gamepad1.getLeftX(),
//                        gamepad1.getLeftY(),
//                        gamepad1.getRightY()
//                );
//                break;
//
//            default:
//                driveType = DriveControls.ARCADE;
//        }
//    }

//    private void left(double power) {
//        try {
//            frontLeft.set(power);
//            backLeft.set(power);
//        } catch(Exception ex) {}
//    }
//
//    private void right(double power) {
//        try {
//            frontRight.set(power);
//            backRight.set(power);
//        } catch(Exception ex) {}
//    }
//
//    public void drive(double leftPower, double rightPower) {
//        left(leftPower);
//        right(rightPower);
//    }
//
//    public void strafe(double power) {
//        frontLeft.set(power);
//        backLeft.set(-power);
//        frontRight.set(-power);
//        backRight.set(power);
//    }
//
//    public void turn (double power) {
//        frontLeft.set(power);
//        backLeft.set(power);
//        frontRight.set(-power);
//        backRight.set(-power);
//    }

}
=======
    private void left(double power) {
        try {
            frontLeft.setPower(power);
            backLeft.setPower(power);
        } catch(Exception ex) {}
    }

    private void right(double power) {
        try {
            frontRight.setPower(power);
            backRight.setPower(power);
        } catch(Exception ex) {}
    }

    public void drive(double leftPower, double rightPower) {
        left(leftPower);
        right(rightPower);
    }

    public void strafe(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(power);
    }

    public void turn (double power) {

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }

}
>>>>>>> 38dc0cfd9297e2c84fe2a488cfec77976a89c00f
