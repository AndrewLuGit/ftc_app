package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Autonomous for Relic Recovery
 */
@Autonomous(name="Mechanum Autonomous",group="mechanum")
public class mech_auto extends LinearOpMode {
    private DcMotor drivelf;
    private DcMotor driverf;
    private DcMotor drivelb;
    private DcMotor driverb;
    private CRServo grabber;
    private Servo jewelHitter;
    private LynxI2cColorRangeSensor colorRange;
    private BNO055IMU imu;
    private Orientation angles;
    private double heading;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    @Override
    public void runOpMode() throws InterruptedException {
        drivelf = hardwareMap.dcMotor.get("drivelf");
        driverf = hardwareMap.dcMotor.get("driverf");
        drivelb = hardwareMap.dcMotor.get("drivelb");
        driverb = hardwareMap.dcMotor.get("driverb");
        driverf.setDirection(DcMotor.Direction.REVERSE);
        driverb.setDirection(DcMotor.Direction.REVERSE);
        grabber = hardwareMap.crservo.get("grabber");
        jewelHitter = hardwareMap.servo.get("jewelHitter");
        colorRange = (LynxI2cColorRangeSensor) hardwareMap.get("color");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        VuforiaLocalizer.Parameters paramters2 = new VuforiaLocalizer.Parameters();
        paramters2.vuforiaLicenseKey = "AVpbLJb/////AAAAGXZuk17KREdul0PqldXjI4ETC+yUOY/0Kn2QZcusavTR02WKxGvyI4E5oodS5Jta30WYJtnJuH7AhLaMe8grr9UC2U3qlnQkypIAZsR8xa38f669mVIo9wujvkZpHzvscPZGdZ2NaheUepxU/asMbuldnDOo3TjSYiiEbk1N3OkxdTeMa4W+BOyrO6sD8L7bcPfnFpmuOPRv0+NeEUL638AjNyi+GQeHYaSLsu6u4ONKtwF+axjjg0W+LRgp5T/5oWxexW3fgoMrkijzsJ0I5OuxSdCeZ3myJthxcyHwHqdhuxmWFvFOoYgJ4k6LdGNijymNWqMp97utjg8YXMAguMLJU2QkPJvZQqbkzIdjzzQk";
        paramters2.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(paramters2);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
        jewelHitter.setPosition(0.34);
        telemetry.addLine("Init Ready");
        telemetry.update();
        /* start of the code */
        waitForStart();
        // Clear telemetry
        telemetry.clear();
        /* lower jewel hitter, wait until in position */
        telemetry.addData("Servo Position",jewelHitter.getPosition());
        telemetry.update();
        sleep(1000);
        jewelHitter.setPosition(0.85);
        while (jewelHitter.getPosition()!=0.85) {
            sleep(500);
            telemetry.addData("Servo Position",jewelHitter.getPosition());
            telemetry.update();
        }
        sleep(500);
        /* Detect the color */
        if (colorRange.blue()>colorRange.red()) {
            imudrive(-0.5,0.5,-0.5,0.5,15);
            jewelHitter.setPosition(0.34);
            while (jewelHitter.getPosition()!=0.34) {
                sleep(500);
            }
            imudrive(0.5,-0.5,0.5,-0.5,0);
        } else if (colorRange.red()>colorRange.blue()) {
            imudrive(0.5,-0.5,0.5,-0.5,-15);
            jewelHitter.setPosition(0.34);
            while (jewelHitter.getPosition()!=0.34) {
                sleep(500);
            }
            imudrive(-0.5,0.5,-0.5,0.5,0);
        } else {
            jewelHitter.setPosition(0.34);
            while (jewelHitter.getPosition()!=0.34) {
                sleep(500);
            }
        }
        /* scan the pictograph */
        /*RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (vuMark==RelicRecoveryVuMark.UNKNOWN){
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("vuMark",RelicRecoveryVuMark.from(relicTemplate));
            telemetry.update();
        }
        /*drive to cryptobox */
        //drivetime(-0.5,-0.5,-0.5,-0.5,1000);
        /*if (vuMark==RelicRecoveryVuMark.LEFT){

        } else if (vuMark==RelicRecoveryVuMark.CENTER){

        } else if (vuMark==RelicRecoveryVuMark.RIGHT) {

        }*/
        /*score*/
        imu.stopAccelerationIntegration();
    }
    private void imudrive(double lfPower,double rfPower, double lbPower, double rbPower,double turnDegrees){
        if (heading>turnDegrees) {
            while (heading>turnDegrees){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
                drivelf.setPower(lfPower);
                driverf.setPower(rfPower);
                drivelb.setPower(lbPower);
                driverb.setPower(rbPower);
                telemetry.addData("Heading",heading);
                telemetry.update();
            }
        } else if (heading<turnDegrees) {
            while (heading<turnDegrees){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
                drivelf.setPower(lfPower);
                driverf.setPower(rfPower);
                drivelb.setPower(lbPower);
                driverb.setPower(rbPower);
                telemetry.addData("Heading",heading);
                telemetry.update();
            }
        }
        drivelf.setPower(0.0);
        driverf.setPower(0.0);
        drivelb.setPower(0.0);
        driverb.setPower(0.0);
    }
    private void drivetime(double lfPower,double rfPower, double lbPower, double rbPower,long milliseconds){
        drivelf.setPower(lfPower);
        driverf.setPower(rfPower);
        drivelb.setPower(lbPower);
        driverb.setPower(rbPower);
        sleep(milliseconds);
        drivelf.setPower(0.0);
        driverf.setPower(0.0);
        drivelb.setPower(0.0);
        driverb.setPower(0.0);
    }
}
