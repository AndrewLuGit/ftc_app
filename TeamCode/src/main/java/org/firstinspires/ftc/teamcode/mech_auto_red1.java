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
public class mech_auto_red1 extends LinearOpMode {
    private DcMotor drivelf;
    private DcMotor driverf;
    private DcMotor drivelb;
    private DcMotor driverb;
    private CRServo grabber;
    private Servo jewelHitter;
    private LynxI2cColorRangeSensor colorRange;
    private BNO055IMU imu;
    private Orientation angles;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private boolean myTeamRed = true;
    private int balanceStonePosition = 1;
    private int myPitLocation = 0;    /* 1 : lfts 0: center -1 : right */

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
        paramters2.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(paramters2);

        jewelHitter.setPosition(0.05);
        telemetry.addLine("Init Ready");
        telemetry.update();
        /* start of the code */
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Clear telemetry
        telemetry.clear();
        grabber.setPower(-0.1);
        /* lower jewel hitter, wait until in position */
        jewelHitter.setPosition(0.55);

        kickOpponentJewel(myTeamRed);
        adjustPostionForImageScan(balanceStonePosition);
        /* get my pit location by scan the Vulmark */
        updateMyPitLocation();
        /* get to the right postion before unload Glyphs */
        scorePositioning();
        /* unloading */
        scoreGlyphs();
        /*score*/
        imu.stopAccelerationIntegration();
    }

    private void imudrive(double turnDegrees,double k1){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initDegrees = angles.firstAngle;
        double pwr = 0;
        double currDegrees = angles.firstAngle;
        double tarDegrees = initDegrees + turnDegrees;
        while (Math.abs(tarDegrees-currDegrees)>5){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currDegrees = angles.firstAngle;
            if (turnDegrees==0) {
                break;
            }
            if (Math.abs(turnDegrees)==turnDegrees){
                pwr = k1*(tarDegrees-currDegrees)/(tarDegrees-initDegrees);
                if (pwr<0.2) {
                    pwr=0.2;
                }
            } else if (Math.abs(turnDegrees)==-turnDegrees) {
                pwr = -k1*(tarDegrees-currDegrees)/(tarDegrees-initDegrees);
                if (pwr>-0.2) {
                    pwr=-0.2;
                }
            }
            drivelf.setPower(-pwr);
            driverf.setPower(pwr);
            drivelb.setPower(-pwr);
            driverb.setPower(pwr);
            telemetry.addData("Degrees1",tarDegrees-currDegrees);
            telemetry.addData("Power",pwr);
            telemetry.addData("Degrees",currDegrees);
            telemetry.update();
        }
        drivelf.setPower(0.0);
        driverf.setPower(0.0);
        drivelb.setPower(0.0);
        driverb.setPower(0.0);
        telemetry.clear();
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

    private void kickLeft(boolean isLeft)
    {
        if (isLeft) {
            telemetry.addLine("kick left");
            imudrive(-15,0.3);
            sleep(500);
            imudrive(15,0.3);
        } else {
            telemetry.addLine("kick right");
            imudrive(15,0.3);
            sleep(500);
            imudrive(-15,0.3);
        }
    }

    private boolean redAtRight() {
        if (colorRange.red()>colorRange.blue()) {
            telemetry.addLine("red at right");
            return true;
        } else {
            telemetry.addLine("red at left");
            return false;
        }
    }

    private void kickOpponentJewel(boolean teamRed) {
         /* Detect the color */
        jewelHitter.setPosition(0.05);
        /* ? really need sleep */
        sleep(500);
        if (teamRed) {
            telemetry.addLine("Team red");
            /* kick blue */
            if (redAtRight()) {
                kickLeft(true);
            } else {
                kickLeft(false);
            }
        } else { /*kick red */
            telemetry.addLine("Team blue");
            if (redAtRight()) {
                kickLeft(false);
            } else {
                kickLeft(true);
            }
        }
        telemetry.update();
    }

    private void adjustPostionForImageScan(int bsPosition) {
        drivetime(0.5,0.5,0.5,0.5,300);
    }

    private void updateMyPitLocation() {
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        telemetry.addLine("Scanning");
        telemetry.update();

        while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            /* may need sleep */
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark==RelicRecoveryVuMark.LEFT){
            telemetry.addLine("Left");
            myPitLocation = 1;
        } else if (vuMark==RelicRecoveryVuMark.CENTER){
            telemetry.addLine("Center");
            myPitLocation = 0;
        } else if (vuMark==RelicRecoveryVuMark.RIGHT) {
            telemetry.addLine("Right");
            myPitLocation = -1;
        }
        telemetry.update();
    }

    private void locateCryptoBoxCenter(int bsPosition) {

         /*drive to cryptobox */
        drivetime(1.0,1.0,1.0,1.0,1000);
        imudrive(90,0.5);
        drivetime(-0.5,-0.5,-0.5,-0.5,1000);
    }

    private void adjustScoreAngel(int bsPosition) {

    }

    private void adjustPitLocation(int pitLocation) {

    }
    private void scorePositioning() {
        locateCryptoBoxCenter(balanceStonePosition);
        adjustScoreAngel(balanceStonePosition);
        adjustPitLocation(myPitLocation);
    }

    private void scoreGlyphs() {
        imudrive(-90,0.5);
        drivetime(0.5,0.5,0.5,0.5,500);
        grabber.setPower(0.5);
        sleep(1000);
        grabber.setPower(-0.01);
        drivetime(-0.5,-0.5,-0.5,-0.5,300);
    }
}
