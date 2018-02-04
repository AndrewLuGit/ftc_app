package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.IOException;

import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;

public class myRobot {
    LinearOpMode myOpMode;
    private DcMotor drivelf;
    private DcMotor driverf;
    private DcMotor drivelb;
    private DcMotor driverb;
    private Servo jewelHitter;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private DcMotor glyphDumper;
    private Servo glyphLifter;
    private LynxI2cColorRangeSensor colorRange;
    private BNO055IMU imu;
    private Orientation angles;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private BNO055IMU.AccelerationIntegrator myIntegrator;
    private Position startPosition = null;
    private Position targetPosition = null;
    private boolean myTeamRed = true;
    private int myBSPosition = 2; /* 1: red 1 2: red 2  3: blue 1 4:blue 2*/
    private int myPictoLocation = 0;    /* 1 : left 0: center -1 : right */

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.7;
    private double          drive_speed =   0.7;
    static final double     TURN_SPEED              = 0.6;
    static final double     FULL_SPEED              = 1.0;
    static final double     MY_K1              = 0.20; // Jan 4th gary change MY_K1 = 0.17  1/2/2018 gary change 2 1/2/2018 MY_K1 = 0.19 to make robot go farther

    public void initialize (int bsPosition, boolean isRed, LinearOpMode opMode) {

        myOpMode = opMode;
        try {
            Logging.setup();
            Logging.log("Start Logging");
        } catch (IOException e1) {

            myOpMode.telemetry.addLine("Init Logging failed");
        }
        myBSPosition = bsPosition;
        myTeamRed =   isRed;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        VuforiaLocalizer.Parameters paramters2 = new VuforiaLocalizer.Parameters();

        myOpMode.telemetry.addLine("Init dcMotor");
        myOpMode.telemetry.addLine("get dirve");
        drivelf = myOpMode.hardwareMap.dcMotor.get("drivelf");
        driverf = myOpMode.hardwareMap.dcMotor.get("driverf");
        drivelb = myOpMode.hardwareMap.dcMotor.get("drivelb");
        driverb = myOpMode.hardwareMap.dcMotor.get("driverb");
        intakeLeft = myOpMode.hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = myOpMode.hardwareMap.get(DcMotor.class, "intakeRight");
        glyphDumper = myOpMode.hardwareMap.get(DcMotor.class,"glyphDumper");
        glyphLifter = myOpMode.hardwareMap.get(Servo.class,"glyphLifter");
        drivelf.setDirection(DcMotor.Direction.REVERSE);
        drivelb.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        glyphDumper.setDirection(DcMotor.Direction.REVERSE);
        myOpMode.telemetry.addLine("reset encode");

        drivelf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driverf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivelb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driverb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivelf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driverf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivelb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driverb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphDumper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        myOpMode.telemetry.addLine("Init IMU");
        myIntegrator = new FineAccelerationIntegrator();
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = myIntegrator;
        imu = myOpMode.hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        jewelHitter = myOpMode.hardwareMap.servo.get("jewelHitter");
        colorRange = (LynxI2cColorRangeSensor) myOpMode.hardwareMap.get("color");
        myOpMode.telemetry.addLine("Init Vuforia");
        paramters2.vuforiaLicenseKey = "AVpbLJb/////AAAAGXZuk17KREdul0PqldXjI4ETC+yUOY/0Kn2QZcusavTR02WKxGvyI4E5oodS5Jta30WYJtnJuH7AhLaMe8grr9UC2U3qlnQkypIAZsR8xa38f669mVIo9wujvkZpHzvscPZGdZ2NaheUepxU/asMbuldnDOo3TjSYiiEbk1N3OkxdTeMa4W+BOyrO6sD8L7bcPfnFpmuOPRv0+NeEUL638AjNyi+GQeHYaSLsu6u4ONKtwF+axjjg0W+LRgp5T/5oWxexW3fgoMrkijzsJ0I5OuxSdCeZ3myJthxcyHwHqdhuxmWFvFOoYgJ4k6LdGNijymNWqMp97utjg8YXMAguMLJU2QkPJvZQqbkzIdjzzQk";
        paramters2.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.createVuforiaLocalizer(paramters2);

        myOpMode.telemetry.addLine("Init Vuforia");

        jewelHitter.setPosition(0.075);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 20);
        myOpMode.telemetry.addLine("Init Ready");
        myOpMode.telemetry.update();
        Logging.log("Init succeed!");
    }

    public void run() {
         jewelHitter.setPosition(0.52);
         myOpMode.sleep(500);
         jewelHitter.setPosition(0.6);
         myOpMode.sleep(100);
         kickOpponentJewel(myTeamRed);

         /* get my pit location by scan the Vumark */
         updateMyPitLocation();

          /* get to the right postion before unload Glyphs */
          scorePositioning();

          /* unloading */
          scoreGlyphs();
          fuzzy();
          push();
    }

    public void fast_run() {
             drive_speed = 0.8;
             jewelHitter.setPosition(0.52);
             myOpMode.sleep(500);
             jewelHitter.setPosition(0.6);
             myOpMode.sleep(100);
             kickOpponentJewel(myTeamRed);

              /* get my pit location by scan the Vumark */
             updateMyPitLocation();
               /* get to the right postion before unload Glyphs */
             scorePositioning();
               /* unloading */
             scoreGlyphs();
            // fuzzy();
             push();
             second_pick();
             scoreGlyphs();
         }

    public void stop() {
        stop_auto();
        imu.stopAccelerationIntegration();
    }





    private void encoderDrive(double speed,
                              double leftInches,
                              double rightInches,
                              double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

       /* reset encode counter */
        drivelf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driverf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivelb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driverb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ElapsedTime runtime = new ElapsedTime();
        drivelf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driverf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivelb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driverb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Determine new target position, and pass to motor controller
        newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = (int) (rightInches * COUNTS_PER_INCH);
        Logging.log("Front wheel Running to %7d :%7d", newLeftTarget, newRightTarget);

        drivelf.setTargetPosition(newLeftTarget);
        drivelb.setTargetPosition(newLeftTarget);
        driverf.setTargetPosition(newRightTarget);
        driverb.setTargetPosition(newRightTarget);


        // reset the timeout time and start motion.
        runtime.reset();
        drivelf.setPower(Math.abs(speed));
        driverf.setPower(Math.abs(speed));
        drivelb.setPower(Math.abs(speed));
        driverb.setPower(Math.abs(speed));


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        while (myOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (drivelf.isBusy() && driverf.isBusy())) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Path2", "Running at %7d :%7d",
                    drivelf.getCurrentPosition(),
                    driverf.getCurrentPosition());
            myOpMode.telemetry.update();
            Logging.log("Front wheel Running at %7d :%7d", drivelf.getCurrentPosition(), driverf.getCurrentPosition());
            Logging.log("back wheel Running at %7d :%7d", drivelb.getCurrentPosition(), driverb.getCurrentPosition());
        }

        // Stop all motion;
        drivelf.setPower(0);
        driverf.setPower(0);
        drivelb.setPower(0);
        driverb.setPower(0);

    }

    /* The following function is to revert normalize happening in library,  it is not desired
        * revert here.  here is the normalize function
        * static double normalizeDegrees(double degrees)
        * {
        *  while (degrees >= 180.0) degrees -= 360.0;
        * while (degrees < -180.0) degrees += 360.0;
        * return degrees;
        *  }
        *
        */
    private double revert_normalize(double initDegrees, double currDegrees) {
        if (initDegrees > 135 && currDegrees < 0) {
            currDegrees += 360;
        } else if (initDegrees < -135 && currDegrees > 0) {
            currDegrees -= 360;
        }
        return currDegrees;
    }

    private void imudrive(double turnDegrees, double k1) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initDegrees = angles.firstAngle;
        double pwr = 0;
        double currDegrees = angles.firstAngle;
        double tarDegrees = initDegrees + turnDegrees;
        double degree_offset;
        double drive_distance = 0;
        int count = 0;

        degree_offset = tarDegrees - currDegrees;
        Logging.log("drive distance, %.3f", currDegrees);
        Logging.log("drive distance, %.3f", degree_offset);
        while (myOpMode.opModeIsActive() && (Math.abs(degree_offset) > 0.6)) {
            if (++count > 4)
                break;
            drive_distance = (degree_offset * k1);
            Logging.log("drive distance, %.3f", drive_distance);
            myOpMode.telemetry.addData("drive distance", drive_distance);
            encoderDrive(TURN_SPEED, -drive_distance, drive_distance, 5.0);  // S2: Turn Right 12 Inches with 5Sec timeout
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currDegrees = revert_normalize(currDegrees, angles.firstAngle);
            degree_offset = tarDegrees - currDegrees;
            myOpMode.telemetry.addData("current degree", currDegrees);
            myOpMode.telemetry.addData("Degrees_offset", degree_offset);
            Logging.log("drive distance, %.3f", currDegrees);
            Logging.log("drive distance, %.3f", degree_offset);
            myOpMode.telemetry.update();
        }
    }

    private void imudrive_target(double tarDegrees, double k1) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initDegrees = angles.firstAngle;
        double pwr = 0;
        double currDegrees = angles.firstAngle;
        double degree_offset;
        double drive_distance = 0;
        int count = 0;

        degree_offset = tarDegrees - currDegrees;
        Logging.log("drive distance, %.3f", currDegrees);
        Logging.log("drive distance, %.3f", degree_offset);
        while (myOpMode.opModeIsActive() && (Math.abs(degree_offset) > 0.6)) {
            if (++count > 5)
                break;
            drive_distance = (degree_offset * k1);
            Logging.log("drive distance, %.3f", drive_distance);
            myOpMode.telemetry.addData("drive distance", drive_distance);
            encoderDrive(TURN_SPEED, -drive_distance, drive_distance, 5.0);  // S2: Turn Right 12 Inches with 5Sec timeout
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currDegrees = revert_normalize(currDegrees, angles.firstAngle);
            degree_offset = tarDegrees - currDegrees;
            myOpMode.telemetry.addData("current degree", currDegrees);
            myOpMode.telemetry.addData("Degrees_offset", degree_offset);
            Logging.log("drive distance, %.3f", currDegrees);
            Logging.log("drive distance, %.3f", degree_offset);
            myOpMode.telemetry.update();
        }
    }

    private void drivetime(double lfPower, double rfPower, double lbPower, double rbPower, long milliseconds) {
        drivelf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driverf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivelf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driverf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivelf.setPower(lfPower);
        driverf.setPower(rfPower);
        drivelb.setPower(lbPower);
        driverb.setPower(rbPower);
        myOpMode.sleep(milliseconds);
        drivelf.setPower(0.0);
        driverf.setPower(0.0);
        drivelb.setPower(0.0);
        driverb.setPower(0.0);
    }



    private void kickLeft(boolean isLeft) {
        if (isLeft) {
            myOpMode.telemetry.addLine("kick left");
            encoderDrive(TURN_SPEED, -3, 3, 4.0);
            jewelHitter.setPosition(0.05);
            myOpMode.sleep(200);
            encoderDrive(TURN_SPEED, 3, -3, 4.0);
        } else {
            myOpMode.telemetry.addLine("kick right");
            encoderDrive(TURN_SPEED, 3, -3, 4.0);
            jewelHitter.setPosition(0.05);
            myOpMode.sleep(200);

            encoderDrive(TURN_SPEED, -3, 3, 4.0);
        }
    }
    private boolean colorValueValid(int val) {
        if (val >= 8 && val <= 250  ) {
            return true;
        } else {
            return false;
        }
    }

    private int  redAtRight() {
        myOpMode.sleep(200);
        myOpMode.telemetry.addLine("red " + colorRange.red() + " blue: " + colorRange.blue());

        /* check whether value is valid or not */
        if (!colorValueValid(colorRange.red()) || !colorValueValid(colorRange.blue())) {
            return -1;
        }

        if (colorRange.red() < colorRange.blue()) {
           myOpMode.telemetry.addLine("red at right");
            myOpMode.sleep(20);
            if (colorRange.red() < colorRange.blue()) {
                return 1;
            }
        } else if (colorRange.red() > colorRange.blue()){
            myOpMode.telemetry.addLine("red at left");
            myOpMode.sleep(20);
            if (colorRange.red() > colorRange.blue()) {
                return 0;
            }
            return 0;
        }
        myOpMode.telemetry.update();
        return -1;
    }

    private void kickOpponentJewel(boolean teamRed) {
         /* Detect the color */
        if (teamRed) {
            myOpMode.telemetry.addLine("0101 Team red");
            /* kick blue */
            final int rar = redAtRight();
            myOpMode.telemetry.addLine("color value: " + rar);
            if (rar == 1) {
                kickLeft(true);
            } else if (rar == 0) {
                kickLeft(false);
            } else {
                jewelHitter.setPosition(0.05);
            }
        } else { /*kick red */
            myOpMode.telemetry.addLine("0101 Team blue");
            if (redAtRight() == 1) {
                kickLeft(false);
            } else if (redAtRight() == 0) {
                kickLeft(true);
            } else {
                jewelHitter.setPosition(0.05);
            }
        }
        myOpMode.telemetry.update();
    }


    private void updateMyPitLocation() {
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        RelicRecoveryVuMark vuMark1;
        int offset = 0;
        myOpMode.telemetry.addLine("Scanning");
        myOpMode.telemetry.update();

        while (vuMark == RelicRecoveryVuMark.UNKNOWN) {

            if (offset < -4) {
                myPictoLocation = 0;
                myOpMode.telemetry.addData("can't figure out VuMark", offset);
                myOpMode.telemetry.update();
                break;
            }
            encoderDrive(TURN_SPEED, -2, 2, 1.0);
            myOpMode.sleep(200);
            offset = offset - 2;
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        myOpMode.telemetry.addData("VuMark offset", offset);
        myOpMode.telemetry.update();
        vuMark1 = RelicRecoveryVuMark.from(relicTemplate);

        while (vuMark != vuMark1) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            vuMark1 = RelicRecoveryVuMark.from(relicTemplate);
        }

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            myPictoLocation = 1;
            myOpMode.telemetry.addData("get position left", myPictoLocation);
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            myPictoLocation = 0;
            myOpMode.telemetry.addData("get position center", myPictoLocation);
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            myPictoLocation = -1;
            myOpMode.telemetry.addData("get position right", myPictoLocation);
        }
        myOpMode.telemetry.update();
        encoderDrive(TURN_SPEED, -offset, offset, 4.0);
        imudrive_target(0, MY_K1);
        myOpMode.telemetry.addData("VuMark position", myPictoLocation);
        myOpMode.telemetry.update();
    }

    private double LocationOffset() {
        double offset = 0;

        if (myBSPosition == 1 || myBSPosition == 2) {
            offset = 7.6;
        } else if (myBSPosition == 3 || myBSPosition == 4) {
            offset = -7.6;
        }

        if (myPictoLocation == -1) {
            offset = 0 - offset;
        } else if (myPictoLocation == 0) {
            offset = 0;
        }
        myOpMode.telemetry.addData("LocationOffset", offset);
        myOpMode.telemetry.update();
        return offset;

    }

    private void scorePositioning() {

         /*drive to cryptobox */
        double offset = LocationOffset();
        double to_center;
        double distance_h = 0;
        double distance_v = 0;
        double w_center_offset = 0;
        double bsoffset1 = 0.5;   /* forwarding direction */
        double bsoffset2 = 0;  /* jewel hit direction, make it longer?*/
        double to_cryptbox = 6.00;

        if (myBSPosition == 1 || myBSPosition == 3) {

            to_center = 12;
            if (myBSPosition == 1) {

                distance_h = (to_center + offset + bsoffset2 - w_center_offset);
                distance_v = 24 + bsoffset1 + w_center_offset;
                to_cryptbox =  5.0 +  w_center_offset - bsoffset2;
                encoderDrive(drive_speed, distance_v, distance_v, 4);
                imudrive_target(90, MY_K1);
                encoderDrive(drive_speed, distance_h, distance_h, 4);
                imudrive_target(0, MY_K1);

            } else {
                distance_h = (to_center + offset  - w_center_offset);
                distance_v = -(24.5 - bsoffset1 + w_center_offset);
                to_cryptbox =  to_cryptbox  -  w_center_offset - bsoffset2;
                encoderDrive(drive_speed, distance_v, distance_v, 4);
                imudrive_target(90, MY_K1);
                encoderDrive(drive_speed, distance_h, distance_h, 4);
                imudrive_target(180, MY_K1);
            }
        }
        if (myBSPosition == 2 || myBSPosition == 4) {
            to_center = 36;
            if (myBSPosition == 2) {
                distance_h = (to_center + offset + bsoffset1 - w_center_offset);
                to_cryptbox =  to_cryptbox +  w_center_offset - bsoffset2 + 0.5;
            } else {
                distance_h = -(to_center + offset - bsoffset1 + w_center_offset+ 0.5);
                to_cryptbox =  to_cryptbox -  w_center_offset - bsoffset2;
            }
            encoderDrive(drive_speed, distance_h, distance_h, 4.0);
            imudrive_target(-90, MY_K1);
        }
        encoderDrive(drive_speed, to_cryptbox, to_cryptbox, 5.0);
    }

    private void scoreGlyphs() {
        /* touch the door */
        myOpMode.sleep(200);
        /* leaving  enough space for drop the Glyph*/
       // encoderDrive(DRIVE_SPEED,-5,-5,2);
        /* move Glyph out of convey belt for easy lifting */
        intakeLeft.setPower(0.4);
        intakeRight.setPower(0.4);
        myOpMode.sleep(300);
        /* start shooting, initially with bigger power then slow down */
        //glyphLifter.setPosition(0.0);
        glyphDumper.setPower(0.4);
        myOpMode.sleep(1200);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        glyphDumper.setPower(0.1);
    }

    public void fuzzy() {
          encoderDrive(DRIVE_SPEED,-2,2,1.0);
          encoderDrive(DRIVE_SPEED,-1.5,-1.5,1.0);
          encoderDrive(DRIVE_SPEED,3,-3,1.0);
    }

    public void push() {
            encoderDrive(DRIVE_SPEED,-3,-3,1.0);
            glyphDumper.setPower(-0.5);
            encoderDrive(DRIVE_SPEED,6,6,1.5);
    }

    private void stop_auto() {
         glyphDumper.setPower(0);
         encoderDrive(DRIVE_SPEED,-4,-4,1.0);

    }

    private void second_pick() {
        intakeLeft.setPower(1);
        intakeRight.setPower(1);
        encoderDrive(FULL_SPEED,-40,-40,10.0);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        imudrive_target(-90, MY_K1);
        encoderDrive(FULL_SPEED,37,37,10);
        scoreGlyphs();
    }
}