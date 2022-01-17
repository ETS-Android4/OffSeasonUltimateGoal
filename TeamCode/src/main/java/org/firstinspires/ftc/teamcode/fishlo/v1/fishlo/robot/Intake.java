package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

import java.time.OffsetDateTime;

public class Intake extends SubSystem {

    DcMotor arm;
    DcMotor intake;
    DcMotor duck;

    Servo capstoneClaw;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime duckTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    double[] position = {0, 0.5};
    int positionIndex = 0;

    int cpr = 28;
    double gearRatio = 19.2;
    double diameter = DriveConstants.WHEEL_RADIUS * 2;
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);
    double bias = 0.8;
    double meccyBias = 0.9;
    double conversion = cpi * bias;

    /**
     * Construct a subsystem with the robot it applies to.
     *
     * @param robot
     */
    public Intake(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        arm = robot.hardwareMap.dcMotor.get("arm");
        intake = robot.hardwareMap.dcMotor.get("intake");
        duck = robot.hardwareMap.dcMotor.get("carousel");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        capstoneClaw = robot.hardwareMap.servo.get("capstoneClaw");
    }

    @Override
    public void handle() {
        arm.setPower(-robot.gamepad2.right_stick_y/2);
        arm.setPower(-robot.gamepad2.left_stick_y);
        double increment = 0.1;
        if (robot.gamepad2.right_bumper) {
            duck.setPower(-0.8);
        }
        if (robot.gamepad2.a) {
            capstoneClaw.setPosition(0.5);
        }
        else if (robot.gamepad2.b) {
            capstoneClaw.setPosition(0);
        }
        else if (robot.gamepad2.y) {
            capstoneClaw.setPosition(1);
        }
        duck.setPower(robot.gamepad2.right_stick_x);

        if (robot.gamepad2.right_trigger >= 0.5) {
            intake(IntakeState.ON);
        }
        else if (robot.gamepad2.left_trigger >= 0.5) {
            intake(IntakeState.REVERSE);
        }
        else {
            intake(IntakeState.OFF);
        }
    }

    public enum IntakeState {
        ON,
        OFF,
        REVERSE
    }

    public void armToLevel(int level) {
        double ticksPerRevHalf = 537.7/2;  //537.7
        double ticksPerRev = 537.7;
        int target = 0;
        if (level == 0) {
            target = 300;
        }
        else if (level == 1) {
            target = 475;
        }
        else if (level == 2) {
            target = 600;
        }
        else if (level == 3) {
            target = 0;
        }
        arm.setTargetPosition(target);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        while (arm.isBusy()) {
            // do nohing
        }
    }

    public void intake(IntakeState state) {
        if (state == IntakeState.OFF) {
            intake.setPower(0);
        }
        else if (state == IntakeState.ON) {
            intake.setPower(1);
        }
        else if (state == IntakeState.REVERSE) {
            intake.setPower(-0.5);
        }
    }

    public void spinCarousel(double power, double time){
        timer.reset();
        duck.setPower(power);
        if (timer.milliseconds() == time) {
            duck.setPower(0);
        }

    }

    public double getMotorPos() {
        return arm.getCurrentPosition();
    }

    public void resetEncoder() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    @Override
    public void stop() {
        arm.setPower(0);
    }

    public void clawToInitPos() {
        capstoneClaw.setPosition(0.1);
    }
}
