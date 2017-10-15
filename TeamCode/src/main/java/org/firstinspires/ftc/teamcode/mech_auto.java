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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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
    //private LynxI2cColorRangeSensor colorRange;
    private BNO055IMU imu;
    private Orientation angles;
    private double heading;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private Position robotPos;
    private Velocity robotVel;
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
        //colorRange = (LynxI2cColorRangeSensor) hardwareMap.get("color");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        imu.getPosition();
        //robotPos.unit = DistanceUnit.CM;
        VuforiaLocalizer.Parameters paramters2 = new VuforiaLocalizer.Parameters();
        paramters2.vuforiaLicenseKey = "AVpbLJb/////AAAAGXZuk17KREdul0PqldXjI4ETC+yUOY/0Kn2QZcusavTR02WKxGvyI4E5oodS5Jta30WYJtnJuH7AhLaMe8grr9UC2U3qlnQkypIAZsR8xa38f669mVIo9wujvkZpHzvscPZGdZ2NaheUepxU/asMbuldnDOo3TjSYiiEbk1N3OkxdTeMa4W+BOyrO6sD8L7bcPfnFpmuOPRv0+NeEUL638AjNyi+GQeHYaSLsu6u4ONKtwF+axjjg0W+LRgp5T/5oWxexW3fgoMrkijzsJ0I5OuxSdCeZ3myJthxcyHwHqdhuxmWFvFOoYgJ4k6LdGNijymNWqMp97utjg8YXMAguMLJU2QkPJvZQqbkzIdjzzQk";
        paramters2.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(paramters2);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        //telemetry.addData("Distance",colorRange.getDistance(DistanceUnit.CM));
        telemetry.update();
        jewelHitter.setPosition(1.0);
        waitForStart();
        /*imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        robotPos = imu.getPosition();
        double initPos = robotPos.y;
        while (robotPos.x != initPos+10){
            robotPos = imu.getPosition();
            drivelf.setPower(0.5);
            driverf.setPower(0.5);
            drivelb.setPower(0.5);
            driverb.setPower(0.5);
        }
        drivelf.setPower(0);
        driverf.setPower(0);
        drivelb.setPower(0);
        driverb.setPower(0);*/
        //Jewel/read pictograph
        jewelHitter.setPosition(0.25);
        sleep(500);
        imudrive(0.5,-0.5,0.5,-0.5,-45);
        sleep(500);
        jewelHitter.setPosition(1.0);
        sleep(500);
        imudrive(-0.5,0.5,-0.5,0.5,45);
        //drive to cryptobox
        //score
        idle();
    }
    private void imudrive(double lfPower,double rfPower, double lbPower, double rbPower,double turnDegrees){
        double turnFinal = heading+turnDegrees;
        while (heading!=turnFinal){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
            drivelf.setPower(lfPower);
            driverf.setPower(rfPower);
            drivelb.setPower(lbPower);
            driverb.setPower(rbPower);
            telemetry.addData("Heading",heading);
            telemetry.update();
        }
        drivelf.setPower(0.0);
        driverf.setPower(0.0);
        drivelb.setPower(0.0);
        driverb.setPower(0.0);
    }
    private void drivetime(double lfPower,double rfPower, double lbPower, double rbPower,long time){
        drivelf.setPower(lfPower);
        driverf.setPower(rfPower);
        drivelb.setPower(lbPower);
        driverb.setPower(rbPower);
        sleep(time);
        drivelf.setPower(0.0);
        driverf.setPower(0.0);
        drivelb.setPower(0.0);
        driverb.setPower(0.0);
    }
}
