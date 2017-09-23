package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Teleop for Mechanum Drive use as teleop
 */
@TeleOp(name="Mechanum Teleop",group="mechanum")
public class mech_teleop extends OpMode {
    /*Declares drive motors: lf = left front,lb = left back,etc.*/
    private DcMotor drivelf;
    private DcMotor driverf;
    private DcMotor drivelb;
    private DcMotor driverb;
    private CRServo grabber;
    @Override
    public void init() {
        drivelf = hardwareMap.dcMotor.get("drivelf");
        driverf = hardwareMap.dcMotor.get("driverf");
        drivelb = hardwareMap.dcMotor.get("drivelb");
        driverb = hardwareMap.dcMotor.get("driverb");
        grabber = hardwareMap.crservo.get("garabber");
        drivelf.setDirection(DcMotorSimple.Direction.REVERSE);
        drivelb.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (Math.abs(gamepad1.left_stick_x)<0.5){
            drivelf.setPower(gamepad1.left_stick_y);
            driverf.setPower(gamepad1.left_stick_y);
            drivelb.setPower(gamepad1.left_stick_y);
            driverb.setPower(gamepad1.left_stick_y);
        }else if (Math.abs(gamepad1.left_stick_x)>=0.5&&Math.abs(gamepad1.left_stick_y)<0.5){
            drivelf.setPower(-gamepad1.left_stick_x);
            driverf.setPower(gamepad1.left_stick_x);
            drivelb.setPower(gamepad1.left_stick_x);
            driverb.setPower(-gamepad1.left_stick_x);
        }else if (gamepad1.left_stick_x>=0.5&&gamepad1.left_stick_y>=0.5){
            drivelb.setPower(Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2)*2);
            drivelf.setPower(0);
            driverb.setPower(0);
            driverf.setPower(Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2)*2);
        }else if (gamepad1.left_stick_x<=-0.5&&gamepad1.left_stick_y>=0.5){
            driverb.setPower(Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2)*2);
            driverf.setPower(0);
            drivelb.setPower(0);
            drivelf.setPower(Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2)*2);
        }else if (gamepad1.left_stick_x>=0.5&&gamepad1.left_stick_y<=-0.5){
            driverb.setPower(-Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2)*2);
            driverf.setPower(0);
            drivelb.setPower(0);
            drivelf.setPower(-Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2)*2);
        }else if (gamepad1.left_stick_x<=-0.5&&gamepad1.left_stick_y<=-0.5){
            drivelb.setPower(-Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2)*2);
            driverb.setPower(0);
            drivelf.setPower(0);
            driverf.setPower(-Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2)*2);
        }
        if (Math.abs(gamepad1.left_stick_x)<=0.5&&Math.abs(gamepad1.left_stick_y)<=0.5&&Math.abs(gamepad1.right_stick_x)>=0.5){
            drivelf.setPower(-gamepad1.right_stick_x);
            driverf.setPower(gamepad1.right_stick_x);
            drivelb.setPower(-gamepad1.right_stick_x);
            driverb.setPower(gamepad1.right_stick_x);
        }
        if(gamepad1.left_bumper){
            grabber.setPower(1.0);
        } else if(gamepad1.left_trigger>=0.5){
            grabber.setPower(-1.0);
        }
    }
}
