package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Autonomous for def2 drive.
 * Currently used to stop on line
 */
@Disabled
@Autonomous(name="square",group="def2")
public class wcauto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor driveLeft;
    private DcMotor driveRight;
    private ColorSensor color1;
    /*private BNO055IMUImpl imu;
    Orientation angles;
    Acceleration gravity;
    */
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMTETER_INCHES = 2.5;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMTETER_INCHES*3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        /*BNO055IMUImpl.Parameters parameters = new BNO055IMUImpl.Parameters();
        parameters.angleUnit = BNO055IMUImpl.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMUImpl.AccelUnit.METERS_PERSEC_PERSEC;
        */
        System.out.println("====Andrew started ====");
        driveLeft = hardwareMap.dcMotor.get("driveLeft");
        System.out.println("====Andrew driveLeft ====");
        driveRight = hardwareMap.dcMotor.get("driveRight");
        System.out.println("====Andrew driveRight ====");
        color1 = hardwareMap.colorSensor.get("color1");
        driveRight.setDirection(DcMotor.Direction.REVERSE);
        System.out.println("====Andrew REVERSE ====");
        telemetry.addData("Status","Resetting Encoders");
        telemetry.update();
        System.out.println("====Andrew update ====");
        driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        System.out.println("====Andrew RUN_USING_ENCODER ====");
        telemetry.addData("Path0","Starting at %7d :%7d",driveLeft.getCurrentPosition(),driveRight.getCurrentPosition());
        telemetry.update();
        waitForStart();
        System.out.println("====Andrew waitForStart ====");
        int calibrate = color1.alpha();
        while(color1.alpha()<20+calibrate){
            driveLeft.setPower(0.1);
            driveRight.setPower(0.1);
            telemetry.addData("color",color1.alpha());
            telemetry.update();
        }
        driveLeft.setPower(0);
        driveRight.setPower(0);
        sleep(1000);
        driveLeft.setPower(0.1);
        driveRight.setPower(-0.1);
        sleep(900);
        driveLeft.setPower(0);
        driveRight.setPower(0);
    }
    public void encoderDrive(double speed, double leftinch, double rightinch, double timeout){
        int leftTarget;
        int rightTarget;
        if(opModeIsActive()){
            leftTarget = driveLeft.getCurrentPosition()+(int)(leftinch*COUNTS_PER_INCH);
            rightTarget = driveRight.getCurrentPosition()+(int)(rightinch*COUNTS_PER_INCH);
            driveLeft.setTargetPosition(leftTarget);
            driveRight.setTargetPosition(rightTarget);
            driveLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            driveLeft.setPower(Math.abs(speed));
            driveRight.setPower(Math.abs(speed));
            while(opModeIsActive() && (runtime.seconds()<timeout)&&(driveLeft.isBusy()&&driveRight.isBusy())){
                telemetry.addData("Path1","Running to %7d :%7d",leftTarget,rightTarget);
                telemetry.addData("Path2","Running at %7d :%7d",driveLeft.getCurrentPosition(),driveRight.getCurrentPosition());
                telemetry.update();
            }
            driveLeft.setPower(0);
            driveRight.setPower(0);
            driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
