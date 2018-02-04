package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Autonomous for Relic Recovery
 */
@Autonomous(name="Red 2",group="mechanum")

public class mech_auto_red2 extends LinearOpMode {
    private myRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.clear();
        robot = new myRobot();
        robot.initialize(2, true, this);
        /* start of the code */
        waitForStart();
        /* lower jewel hitter, wait until in position */

        robot.fast_run();
        robot.stop();
    }
}