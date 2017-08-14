package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Teleop for Mechanum Drive
 */
@Disabled
public class mech_teleop extends OpMode {
    /*Declares drive motors: lf = left front,lb = left back,etc.*/
    private DcMotor drivelf;
    private DcMotor driverf;
    private DcMotor drivelb;
    private DcMotor driverb;
    @Override
    public void init() {
        drivelf = hardwareMap.dcMotor.get("drivelf");
        driverf = hardwareMap.dcMotor.get("driverf");
        drivelb = hardwareMap.dcMotor.get("drivelb");
        driverb = hardwareMap.dcMotor.get("driverb");
        drivelf.setDirection(DcMotorSimple.Direction.REVERSE);
        drivelb.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(Math.abs(gamepad1.left_stick_x)<0.5){
            drivelf.setPower(gamepad1.left_stick_y);
            driverf.setPower(gamepad1.left_stick_y);
            drivelb.setPower(gamepad1.left_stick_y);
            driverb.setPower(gamepad1.left_stick_y);
        }else if(Math.abs(gamepad1.left_stick_x)>=0.5&&Math.abs(gamepad1.left_stick_y)<0.5){
            drivelf.setPower(gamepad1.left_stick_x);
            driverf.setPower(-gamepad1.left_stick_x);
            drivelb.setPower(-gamepad1.left_stick_x);
            driverb.setPower(gamepad1.left_stick_x);
        }else if(gamepad1.left_stick_x>=0.5&&Math.abs(gamepad1.left_stick_y)>=0.5){
            drivelf.setPower(Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2));
            driverb.setPower(Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2));
        }else if(gamepad1.left_stick_x<=-0.5&&Math.abs(gamepad1.left_stick_y)>=0.5){
            driverf.setPower(Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2));
            drivelb.setPower(Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2));
        }else if(gamepad1.left_stick_x>=0.5&&Math.abs(gamepad1.left_stick_y)<=-0.5){
            driverf.setPower(-Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2));
            drivelb.setPower(-Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2));
        }else if(gamepad1.left_stick_x<=-0.5&&Math.abs(gamepad1.left_stick_y)<=-0.5){
            drivelf.setPower(-Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2));
            driverb.setPower(-Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))/Math.sqrt(2));
        }
    }
}
