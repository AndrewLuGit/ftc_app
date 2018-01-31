package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.IOException;

/**
 * Autonomous for Relic Recovery
 */
@Autonomous(name="Red 1",group="mechanum")
public class mech_auto_red1 extends LinearOpMode {
    private myRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.clear();
        robot = new myRobot();
        robot.initialize(1, true, this);
        /* start of the code */
        waitForStart();
        /* lower jewel hitter, wait until in position */

       robot.run();
       robot.stop();
    }
}