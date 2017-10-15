package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Teleop for Mechanum Drive and Relic Recovery
 */
@TeleOp(name="Mechanum Teleop 2",group="mechanum")
public class mech_teleop2 extends OpMode {
    /*Declares drive motors: lf = left front,lb = left back,etc.*/
    private DcMotor drivelf;
    private DcMotor driverf;
    private DcMotor drivelb;
    private DcMotor driverb;
    private CRServo grabber;
    private DcMotor grabberMotor;
    @Override
    public void init() {
        drivelf = hardwareMap.dcMotor.get("drivelf");
        driverf = hardwareMap.dcMotor.get("driverf");
        drivelb = hardwareMap.dcMotor.get("drivelb");
        driverb = hardwareMap.dcMotor.get("driverb");
        grabber = hardwareMap.crservo.get("grabber");
        grabberMotor = hardwareMap.dcMotor.get("grabberMotor");
        drivelf.setDirection(DcMotor.Direction.REVERSE);
        drivelb.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        //Math wizardry
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        drivelf.setPower(v1);
        driverf.setPower(v2);
        drivelb.setPower(v3);
        driverb.setPower(v4);
        if(gamepad1.left_bumper){
            grabber.setPower(1.0);
        } else if(gamepad1.left_trigger>=0.5){
            grabber.setPower(-1.0);
        } else{
            grabber.setPower(-0.01);
        }
        if (gamepad1.right_bumper) {
            grabberMotor.setPower(-1.0);
        } else if (gamepad1.right_trigger >= 0.5) {
            grabberMotor.setPower(1.0);
        } else {
            grabberMotor.setPower(0.0);
        }
        telemetry.addData("grabberPower",grabber.getPower());
        telemetry.update();
    }
}
