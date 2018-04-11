package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *Teleop for postseason robot
 */
@TeleOp(name="Teleop2",group="Mecanum")
public class mech_teleop4 extends LinearOpMode{
    private DcMotor drivelf;
    private DcMotor driverf;
    private DcMotor drivelb;
    private DcMotor driverb;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private DcMotor lift;
    private double k=1;
    @Override
    public void runOpMode() throws InterruptedException {
        drivelf = hardwareMap.get(DcMotor.class, "drivelf");
        driverf = hardwareMap.get(DcMotor.class, "driverf");
        drivelb = hardwareMap.get(DcMotor.class, "drivelb");
        driverb = hardwareMap.get(DcMotor.class, "driverb");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        lift = hardwareMap.get(DcMotor.class, "lift");
        driverf.setDirection(DcMotor.Direction.REVERSE);
        driverb.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        drivelf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driverf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivelb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driverb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()){
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
            if (gamepad1.dpad_up) {
                k = 1;
            } else if (gamepad1.dpad_down) {
                k = 0.3;
            }
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
            } else if (gamepad1.left_trigger>=0.1 || gamepad2.left_trigger>=0.1) {
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
            } else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
            }
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                lift.setPower(1.0);
            } else if (gamepad1.right_trigger>=0.1||gamepad2.right_trigger>=0.1) {
                lift.setPower(-1.0);
            } else {
                lift.setPower(0);
            }
        }
    }
}
