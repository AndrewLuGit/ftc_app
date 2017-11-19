package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Teleop for Mechanum Drive and Relic Recovery
 */
@TeleOp(name="Mechanum Teleop 2",group="mechanum")
public class mech_teleop2 extends LinearOpMode {
    /*Declares drive motors: lf = left front,lb = left back,etc.*/
    private DcMotor drivelf;
    private DcMotor driverf;
    private DcMotor drivelb;
    private DcMotor driverb;
    private CRServo grabber;
    private DcMotor grabberMotor;
    private Servo jewelHitter;
    private double jhPos;
    private double k = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        drivelf = hardwareMap.dcMotor.get("drivelf");
        driverf = hardwareMap.dcMotor.get("driverf");
        drivelb = hardwareMap.dcMotor.get("drivelb");
        driverb = hardwareMap.dcMotor.get("driverb");
        grabber = hardwareMap.crservo.get("grabber");
        grabberMotor = hardwareMap.dcMotor.get("grabberMotor");
        jewelHitter = hardwareMap.servo.get("jewelHitter");
        drivelf.setDirection(DcMotor.Direction.REVERSE);
        drivelb.setDirection(DcMotor.Direction.REVERSE);
        grabberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivelf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driverf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivelb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driverb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jewelHitter.setPosition(0.05);
        waitForStart();
        while (opModeIsActive()){
            //Math wizardry
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        drivelf.setPower(k*v2);
        driverf.setPower(k*v1);
        drivelb.setPower(k*v4);
        driverb.setPower(k*v3);
            if(gamepad1.left_bumper || gamepad2.left_bumper){
                    grabber.setPower(1.0);
            } else if(gamepad1.left_trigger>=0.5 || gamepad2.left_trigger>=0.5){
                grabber.setPower(-1.0);
            } else{
                grabber.setPower(-0.01);
            }
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                grabberMotor.setPower(-0.5);
            } else if (gamepad1.right_trigger >= 0.5 || gamepad2.right_trigger>=0.5) {
                grabberMotor.setPower(0.5);
            } else {
                grabberMotor.setPower(0.0);
            }
            jhPos = jewelHitter.getPosition();
            if (gamepad1.a||gamepad2.a) {
                jewelHitter.setPosition(jhPos+0.01);
            } else if (gamepad1.b||gamepad2.b) {
                jewelHitter.setPosition(jhPos-0.01);
            }
            if (gamepad1.dpad_up) {
                k = 1;
            } else if (gamepad1.dpad_down) {
                k = 0.3;
            }
            telemetry.addData("jhPos",jewelHitter.getPosition());
            telemetry.update();
        }
    }
}
