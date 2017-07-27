package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by garyding on 7/16/17.
 */
@Autonomous(name="testauto")
public class TestAuto extends LinearOpMode{
    private DcMotor driveLeft;
    private DcMotor driveRight;
    private DcMotor armMotor;
    private Servo clawLeft;
    private Servo clawRight;
    @Override
    public void runOpMode() throws InterruptedException {
        driveLeft = hardwareMap.dcMotor.get("driveLeft");
        driveRight = hardwareMap.dcMotor.get("driveRight");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        clawLeft = hardwareMap.servo.get("clawLeft");
        clawRight = hardwareMap.servo.get("clawRight");
        waitForStart();
        //Todo: Make Autonomous part
    }
}
