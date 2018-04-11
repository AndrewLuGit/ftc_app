package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Auto stuff for new robot
 */

public class myRobot2 {
    LinearOpMode myOpMode;
    private DcMotor drivelf;
    private DcMotor driverf;
    private DcMotor drivelb;
    private DcMotor driverb;
    private Servo jewelHitterArm;
    private Servo jewelHitterBase;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private DcMotor lift;
    private JewelDetector jewelDetector = null;
    private BNO055IMU imu;
    private VuforiaLocalizer vuforia;
    VuforiaTrackable pictograph;
    private int myBSPosition;
    private int myPictoLocation=0;
    private boolean myTeamRed;
    public void initialize (int bsPosition, boolean isRed, LinearOpMode opMode) {
        myBSPosition=bsPosition;
        myTeamRed=isRed;
        // Init IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = myOpMode.hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        //Init Vuforia
        VuforiaLocalizer.Parameters paramters2 = new VuforiaLocalizer.Parameters();
        paramters2.vuforiaLicenseKey = "AVpbLJb/////AAAAGXZuk17KREdul0PqldXjI4ETC+yUOY/0Kn2QZcusavTR02WKxGvyI4E5oodS5Jta30WYJtnJuH7AhLaMe8grr9UC2U3qlnQkypIAZsR8xa38f669mVIo9wujvkZpHzvscPZGdZ2NaheUepxU/asMbuldnDOo3TjSYiiEbk1N3OkxdTeMa4W+BOyrO6sD8L7bcPfnFpmuOPRv0+NeEUL638AjNyi+GQeHYaSLsu6u4ONKtwF+axjjg0W+LRgp5T/5oWxexW3fgoMrkijzsJ0I5OuxSdCeZ3myJthxcyHwHqdhuxmWFvFOoYgJ4k6LdGNijymNWqMp97utjg8YXMAguMLJU2QkPJvZQqbkzIdjzzQk";
        paramters2.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.createVuforiaLocalizer(paramters2);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        pictograph = relicTrackables.get(0);
        relicTrackables.activate();
        //Init Drivetrain
        drivelf = myOpMode.hardwareMap.dcMotor.get("drivelf");
        driverf = myOpMode.hardwareMap.dcMotor.get("driverf");
        drivelb = myOpMode.hardwareMap.dcMotor.get("drivelb");
        driverb = myOpMode.hardwareMap.dcMotor.get("driverb");
        drivelf.setDirection(DcMotor.Direction.REVERSE);
        drivelb.setDirection(DcMotor.Direction.REVERSE);
        drivelf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driverf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivelb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driverb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Init Others
        intakeLeft = myOpMode.hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = myOpMode.hardwareMap.get(DcMotor.class, "intakeRight");
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift = myOpMode.hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Init jewelHitter
        jewelHitterArm = myOpMode.hardwareMap.servo.get("jewelHitterArm");
        jewelHitterBase = myOpMode.hardwareMap.servo.get("jewelHitterBase");
        jewelHitterBase.setPosition(0.5);
        jewelHitterArm.setPosition(0);
        //Init DogeCV
        jewelDetector = new JewelDetector();
        jewelDetector.init(myOpMode.hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        jewelDetector.areaWeight =0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;
    }
    public void removeOpponentJewel(boolean teamRed){
        jewelHitterArm.setPosition(0.75);
        jewelDetector.enable();
        myOpMode.sleep(100);
        if (teamRed) {
            if (jewelDetector.getCurrentOrder().equals(JewelDetector.JewelOrder.BLUE_RED)) {
                jewelHitterBase.setPosition(0.35);
                myOpMode.sleep(200);
                jewelHitterBase.setPosition(0.5);
            } else if (jewelDetector.getCurrentOrder().equals(JewelDetector.JewelOrder.RED_BLUE)) {
                jewelHitterBase.setPosition(0.65);
                myOpMode.sleep(200);
                jewelHitterBase.setPosition(0.5);
            }
        } else {
            if (jewelDetector.getCurrentOrder().equals(JewelDetector.JewelOrder.BLUE_RED)) {
                jewelHitterBase.setPosition(0.65);
                myOpMode.sleep(200);
                jewelHitterBase.setPosition(0.5);
            } else if (jewelDetector.getCurrentOrder().equals(JewelDetector.JewelOrder.RED_BLUE)) {
                jewelHitterBase.setPosition(0.35);
                myOpMode.sleep(200);
                jewelHitterBase.setPosition(0.5);
            }
        }
        jewelHitterArm.setPosition(0);
    }
}
