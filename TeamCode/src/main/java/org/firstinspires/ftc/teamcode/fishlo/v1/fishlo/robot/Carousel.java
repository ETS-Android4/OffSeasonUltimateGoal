package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Carousel extends SubSystem {

    DcMotor arm;
    DcMotor intakeWheel;
    DcMotor carouselWheel;

    /**
     * Construct a subsystem with the robot it applies to.
     *
     * @param robot
     */
    public Carousel(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        arm = robot.hardwareMap.dcMotor.get("Arm");
        intakeWheel = robot.hardwareMap.dcMotor.get("intakeWheel");
        carouselWheel = robot.hardwareMap.dcMotor.get("carouselWheel");
    }

    @Override
    public void handle() {
        arm.setPower(robot.gamepad2.right_stick_y);
        arm.setPower((robot.gamepad2.left_stick_y));
        if (robot.gamepad2.a) {
            intakeWheel.setPower(0.5);
        }
    }

    @Override
    public void stop() {

    }
}
