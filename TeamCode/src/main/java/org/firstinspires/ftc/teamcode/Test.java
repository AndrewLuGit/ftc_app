package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Test program for PushBot
 */
@TeleOp(name = "test",group = "test")
public class Test extends OpMode{
    private DcMotor driveLeft;
    private DcMotor driveRight;
    private DcMotor armMotor;
    private Servo clawLeft;
    private Servo clawRight;
    double left;
    double right;
    double max;
    double leftpos;
    double rightpos;
    public static final double clawinit = 0.5;
    public static final double clawspeed = 0.1;
    @Override
    public void init() {
        driveLeft.setDirection(DcMotor.Direction.REVERSE);
        clawLeft.setPosition(clawinit);
        clawRight.setPosition(clawinit);
        driveLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        left = gamepad1.right_stick_y+gamepad1.right_stick_x;
        right = gamepad1.right_stick_y-gamepad1.right_stick_x;
        leftpos = clawLeft.getPosition();
        rightpos = clawRight.getPosition();
        max = Math.max(Math.abs(left),Math.abs(right));
        if(max > 1.0){
            left /= max;
            right /= max;
        }
        driveLeft.setPower(left);
        driveRight.setPower(right);
        if(gamepad1.a){
            leftpos += clawspeed;
            rightpos -= clawspeed;
        } else if(gamepad1.b){
            leftpos -= clawspeed;
            rightpos += clawspeed;
        }
        clawLeft.setPosition(leftpos);
        clawRight.setPosition(rightpos);
        if (gamepad1.right_bumper){
            armMotor.setPower(0.3);
        } else if (gamepad1.right_trigger >= 0.5){
            armMotor.setPower(-0.3);
        } else{
            armMotor.setPower(0.0);
        }
        telemetry.addData("drivePower",driveLeft.getPower());
        telemetry.addData("clawPos",clawLeft.getPosition());
        telemetry.addData("armPower",armMotor.getPower());
        telemetry.update();
    }
}
