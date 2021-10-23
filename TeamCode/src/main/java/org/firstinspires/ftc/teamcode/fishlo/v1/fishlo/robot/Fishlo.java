package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Fishlo extends Robot {

    public Fishlo(OpMode opMode) {
        super(opMode);

        putSubSystem("Drive", new Drive(this));
<<<<<<< HEAD
        putSubSystem("Vision", new Vision(this));
=======
>>>>>>> 38dc0cfd9297e2c84fe2a488cfec77976a89c00f
    }

}