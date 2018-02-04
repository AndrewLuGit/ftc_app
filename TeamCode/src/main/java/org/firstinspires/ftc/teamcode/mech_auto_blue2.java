package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Autonomous for Relic Recovery
 */
@Autonomous(name="Blue 2",group="mechanum")

public class mech_auto_blue2 extends LinearOpMode {
    private myRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.clear();
        robot = new myRobot();
        robot.initialize(4, false, this);
        /* start of the code */
        waitForStart();
        robot.fast_run();
        robot.stop();
    }
}