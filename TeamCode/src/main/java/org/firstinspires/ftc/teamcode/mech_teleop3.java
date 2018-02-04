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
    private DcMotor relicMotor;
    private Servo relicWrist;
    private Servo relicGrabber;
    private LynxI2cColorRangeSensor colorRange;
    private Servo jewelHitter;
    private double k =1;
    private double intakePower;
    private Servo glyphHolder;
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
        jewelHitter = hardwareMap.get(Servo.class,"jewelHitter");
        glyphHolder = hardwareMap.get(Servo.class, "glyphHolder");
        relicMotor = hardwareMap.get(DcMotor.class,"relicMotor");
        relicWrist = hardwareMap.get(Servo.class,"relicWrist");
        relicGrabber = hardwareMap.get(Servo.class, "relicGrabber");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        driverf.setDirection(DcMotor.Direction.REVERSE);
        driverb.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        glyphDumper.setDirection(DcMotorSimple.Direction.REVERSE);
        driverb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivelb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driverf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivelf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Set all motors to brake at zero power mode

        drivelf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driverf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivelb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driverb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphDumper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        jewelHitter.setPosition(0.075);
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
                glyphDumper.setPower(0.4);
                glyphHolder.setPosition(0.45);
            } else if (gamepad1.right_trigger>=0.1 || gamepad2.right_trigger>=0.1) {
                glyphDumper.setPower(-0.4);
            } else {
                glyphDumper.setPower(0);
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                glyphHolder.setPosition(0);
            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                glyphHolder.setPosition(0.45);
            }
            if (gamepad2.dpad_up) {
                relicMotor.setPower(1.0);
            } else if (gamepad2.dpad_down) {
                relicMotor.setPower(-1.0);
            } else {
                relicMotor.setPower(0);
            }
            if (gamepad2.a) {
                relicWrist.setPosition(0.25);
            } else if (gamepad2.b) {
                relicWrist.setPosition(0.75);
            }
            if (gamepad2.x) {
                relicGrabber.setPosition(0.9);
            } else if (gamepad2.y) {
                relicGrabber.setPosition(0.2);
            }
        }
    }
}
