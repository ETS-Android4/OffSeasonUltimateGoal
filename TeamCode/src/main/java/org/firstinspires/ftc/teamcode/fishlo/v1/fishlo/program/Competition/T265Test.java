package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Fishlo;
import org.firstinspires.ftc.teamcode.opMode.DriverControlledProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class T265Test extends DriverControlledProgram {
    private Fishlo fishlo;

    @Override
    protected Robot buildRobot() {
        fishlo = new Fishlo(this);
        return fishlo;
    }
}
