package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Autonomous for Relic Recovery
 */
@Autonomous(name="Blue 1",group="mechanum")

public class mech_auto_blue1 extends LinearOpMode {
    private myRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.clear();
        robot = new myRobot();
        robot.initialize(3, false, this);
        /* start of the code */
        waitForStart();
        robot.run();
        robot.stop();
    }
}