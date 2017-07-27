package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Teleop program for West Coast Drive
 */
@TeleOp(name="Teleop",group="wc")
public class wc extends OpMode {
    private DcMotor driveLeft;
    private DcMotor driveRight;
    public static final double fullSpeed = 1.0;
    public static final double halfSpeed = 0.5;
    private double speed = fullSpeed;
    @Override
    public void init() {
        driveLeft = hardwareMap.dcMotor.get("driveLeft");
        driveRight = hardwareMap.dcMotor.get("driveRight");
        driveLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        driveLeft.setPower(gamepad1.left_stick_y*speed);
        driveRight.setPower(gamepad1.right_stick_y*speed);
        if(gamepad1.dpad_up){
            speed = fullSpeed;
        } else if(gamepad1.dpad_down){
            speed = halfSpeed;
        }
    }
}
