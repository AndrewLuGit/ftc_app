package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Teleop for new 2017-2018 robot
 */

@TeleOp(name="Teleop", group="Mecanum")
public class mech_teleop3 extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor drivelf;
    private DcMotor driverf;
    private DcMotor drivelb;
    private DcMotor driverb;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private DcMotor glyphDumper;
    private LynxI2cColorRangeSensor colorRange;
    private Servo jewelHitter;
    private double k =1;
    private double intakePower;
    private Servo glyphLifter;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        drivelf = hardwareMap.get(DcMotor.class, "drivelf");
        driverf = hardwareMap.get(DcMotor.class, "driverf");
        drivelb = hardwareMap.get(DcMotor.class, "drivelb");
        driverb = hardwareMap.get(DcMotor.class, "driverb");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        glyphDumper = hardwareMap.get(DcMotor.class,"glyphDumper");
        glyphLifter = hardwareMap.get(Servo.class,"glyphLifter");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        drivelf.setDirection(DcMotor.Direction.REVERSE);
        driverb.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        glyphDumper.setDirection(DcMotor.Direction.REVERSE);
        drivelf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driverf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivelb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driverb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Set all motors to brake at zero power mode

        drivelf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driverf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivelb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driverb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphDumper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        jewelHitter.setPosition(0.05);
        drivelf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driverf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivelb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driverb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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
                intakePower = 1;
            } else if (gamepad1.left_trigger>=0.1 || gamepad2.left_trigger>=0.1) {
                intakePower = -1;
            } else {
                intakePower = 0;
            }
            intakeLeft.setPower(intakePower);
            intakeRight.setPower(intakePower);
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                glyphDumper.setPower(0.5);
                glyphLifter.setPosition(1.0);
            } else if (gamepad1.right_trigger>=0.1 || gamepad2.right_trigger>=0.1) {
                glyphDumper.setPower(-0.5);
                glyphLifter.setPosition(0.5);
            } else {
                glyphDumper.setPower(0);
            }
        }
        drivelf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driverf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivelb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driverb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
