package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
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
@Autonomous(name="Red 1",group="mechanum")
public class mech_auto_red1 extends LinearOpMode {
    private DcMotor drivelf;
    private DcMotor driverf;
    private DcMotor drivelb;
    private DcMotor driverb;
    private Servo jewelHitter;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private DcMotor glyphDumper;
    private LynxI2cColorRangeSensor colorRange;
    private BNO055IMU imu;
    private Orientation angles;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private BNO055IMU.AccelerationIntegrator myIntegrator;
    private boolean myTeamRed = true;
    private int myBSPosition = 4; /* 1: top lfts 2: top right 3: bottom lfts 4:bootom right */
    private int myPictoLocation = 0;    /* 1 : lfts 0: center -1 : right */

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.6;

    //------------------------------------------------------------------------------------------------
    // Construction
    //------------------------------------------------------------------------------------------------

   public void initialize () {
       BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
       VuforiaLocalizer.Parameters paramters2 = new VuforiaLocalizer.Parameters();

       telemetry.addLine("Init dcMotor");
       telemetry.addLine("get dirve");
       drivelf = hardwareMap.dcMotor.get("drivelf");
       driverf = hardwareMap.dcMotor.get("driverf");
       drivelb = hardwareMap.dcMotor.get("drivelb");
       driverb = hardwareMap.dcMotor.get("driverb");
       intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
       intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
       glyphDumper = hardwareMap.get(DcMotor.class,"glyphDumper");
       driverf.setDirection(DcMotor.Direction.REVERSE);
       drivelb.setDirection(DcMotor.Direction.REVERSE);
       intakeLeft.setDirection(DcMotor.Direction.REVERSE);
       telemetry.addLine("reset encode");

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
       telemetry.addLine("Init IMU");
       myIntegrator = new FineAccelerationIntegrator();
       parameters = new BNO055IMU.Parameters();
       parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
       parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
       parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
       parameters.loggingEnabled      = true;
       parameters.loggingTag          = "IMU";
       parameters.accelerationIntegrationAlgorithm = myIntegrator;
       imu = hardwareMap.get(BNO055IMU.class,"imu");
       imu.initialize(parameters);

       jewelHitter = hardwareMap.servo.get("jewelHitter");
       colorRange = (LynxI2cColorRangeSensor) hardwareMap.get("color");
       telemetry.addLine("Init Vuforia");
       paramters2.vuforiaLicenseKey = "AVpbLJb/////AAAAGXZuk17KREdul0PqldXjI4ETC+yUOY/0Kn2QZcusavTR02WKxGvyI4E5oodS5Jta30WYJtnJuH7AhLaMe8grr9UC2U3qlnQkypIAZsR8xa38f669mVIo9wujvkZpHzvscPZGdZ2NaheUepxU/asMbuldnDOo3TjSYiiEbk1N3OkxdTeMa4W+BOyrO6sD8L7bcPfnFpmuOPRv0+NeEUL638AjNyi+GQeHYaSLsu6u4ONKtwF+axjjg0W+LRgp5T/5oWxexW3fgoMrkijzsJ0I5OuxSdCeZ3myJthxcyHwHqdhuxmWFvFOoYgJ4k6LdGNijymNWqMp97utjg8YXMAguMLJU2QkPJvZQqbkzIdjzzQk";
       paramters2.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
       vuforia = ClassFactory.createVuforiaLocalizer(paramters2);

       telemetry.addLine("Init Vuforia");

       jewelHitter.setPosition(0.05);
       telemetry.addLine("Init Ready");
       telemetry.update();
   }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.clear();
        initialize();
        /* start of the code */
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 20);
        /* lower jewel hitter, wait until in position */

        jewelHitter.setPosition(0.6);

        while (jewelHitter.getPosition()!=0.6) {
            sleep(100);
            telemetry.addData("Servo Position",jewelHitter.getPosition());
            telemetry.update();
        }
        sleep(200);

        kickOpponentJewel(myTeamRed);
        /* get my pit location by scan the Vumark */
        updateMyPitLocation();

        //test_position_sensor();
        //test_encode();
        //test_moveto();

        /* get to the right postion before unload Glyphs */
        scorePositioning();
        /* unloading */
        scoreGlyphs();
        imu.stopAccelerationIntegration();
        while (opModeIsActive()) {
            telemetry.update();
        }
        imu.stopAccelerationIntegration();
    }


    private  void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        ElapsedTime runtime = new ElapsedTime();

        drivelf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driverf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivelf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driverf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position, and pass to motor controller
        newLeftTarget = drivelf.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = driverf.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        drivelf.setTargetPosition(newLeftTarget);
        driverf.setTargetPosition(newRightTarget);

        newLeftTarget = drivelb.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = driverb.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        drivelb.setTargetPosition(newLeftTarget);
        driverb.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
        drivelf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driverf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivelb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driverb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        while (opModeIsActive()&& (runtime.seconds() < timeoutS) &&
                (drivelf.isBusy() && driverf.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    drivelf.getCurrentPosition(),
                    driverf.getCurrentPosition());
            telemetry.update();
            sleep(10);
        }

        // Stop all motion;
        drivelf.setPower(0);
        driverf.setPower(0);
        drivelb.setPower(0);
        driverb.setPower(0);
            // Turn off RUN_TO_POSITION

        drivelf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driverf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivelb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driverb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drivelf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driverf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivelf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driverf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void test_encode() {
        encoderDrive(DRIVE_SPEED,  24,  24, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
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
        if (initDegrees > 135 && currDegrees < 0  ) {
            currDegrees += 360;
        } else if (initDegrees < -135 && currDegrees > 0) {
            currDegrees -= 360;
        }
        return  currDegrees;
    }

    private void imudrive(double turnDegrees,double k1){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initDegrees = angles.firstAngle;
        double pwr = 0;
        double currDegrees = angles.firstAngle;
        double tarDegrees = initDegrees + turnDegrees;
        double degree_offset;
        double drive_distance = 0;
        int count = 0;

        degree_offset = tarDegrees - currDegrees;
        while (opModeIsActive() && (Math.abs(degree_offset) > 0.6) ){
            if (++count >5)
                break;
            drive_distance  = (degree_offset * k1);

            telemetry.addData("drive distance", drive_distance);
            encoderDrive(TURN_SPEED, -drive_distance , drive_distance, 5.0);  // S2: Turn Right 12 Inches with 5Sec timeout
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currDegrees = revert_normalize(currDegrees, angles.firstAngle);
            degree_offset = tarDegrees - currDegrees;
            telemetry.addData("current degree", currDegrees);
            telemetry.addData("Degrees_offset", degree_offset);
            telemetry.update();
        }
    }

    private void drivetime(double lfPower,double rfPower, double lbPower, double rbPower,long milliseconds){
        drivelf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driverf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivelf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driverf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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


    private void test_imudrive(){

        /* test 45 dergress */
        imudrive(45, 0.17);
        sleep(20);
        imudrive(90, 0.17);
        sleep(20);
        imudrive(45, 0.17);
        sleep(20);
       // imudrive(60, 0.17);
       // sleep(1000);
       // imudrive(-120, 0.17);
       // sleep(1000);
       // imudrive(-100, 0.17);
    }



    private void moveto(Position destination) {
        Position current = imu.getPosition();
        double x_offset = destination.x - current.x;
        double y_offset = destination.y - current.y;
        /* initial imu is on 90 degree position,  so we need switch x_offset with y_offse when
         * caculate turning degree.  whether to moving forward or moving backward depends on x,y
         * postion
         */
        double turn_degree = - Math.atan2(x_offset, y_offset);
        int count = 0;

        double distance =   Math.sqrt(x_offset * x_offset + y_offset * y_offset) / 25.4;

        telemetry.addData("current position",  "%s", current.toString());
        telemetry.addData("destination position",  "%s", destination.toString());
        telemetry.addData("x_offset", x_offset);
        telemetry.addData("y_offset", y_offset);
        telemetry.addData("turn degree", turn_degree);
        telemetry.addData("distance", distance);
        telemetry.update();
        sleep(1000);
        imudrive(turn_degree, 0.17);
        if (y_offset < 0) {
            distance = -distance;
        }

        while (Math.abs(distance) > 0.5) {
            imudrive(turn_degree, 0.17);
            if (++count >3) break;

            telemetry.addData("driving distance", distance);
            encoderDrive(DRIVE_SPEED,  distance, distance , 5.0);
            current = imu.getPosition();
            telemetry.addData("current position",  "%s", current.toString());
            x_offset = (destination.x - current.x);
            y_offset = (destination.y - current.y);
            turn_degree = - Math.atan2(x_offset, y_offset);
            distance = Math.sqrt(x_offset * x_offset + y_offset * y_offset) / 25.4;
            if (y_offset < 0) {
                distance = -distance;
            }
            telemetry.addData("x_offset", x_offset);
            telemetry.addData("y_offset", y_offset);
            telemetry.addData("turn degree", turn_degree);
            telemetry.update();
        }
    }

    private void test_moveto() {
        Position destination = new Position();
        destination.x += 800;
        destination.y += 800;
        destination.unit = DistanceUnit.MM;
        moveto(destination);
    }

    private void test_position_sensor() {
        Position current = imu.getPosition();
        telemetry.addData("current position",  "%s", current.toString());
        encoderDrive(DRIVE_SPEED,  24,  24, 5.0);
        imudrive(90, 0.17);
        encoderDrive(DRIVE_SPEED,  24,  24, 5.0);
        Position destination = imu.getPosition();
        telemetry.addData("new position",  "%s", destination.toString());
        double x_offset = destination.x - current.x;
        double y_offset = destination.y - current.y;
        telemetry.addData("x_offset", x_offset);
        telemetry.addData("y_offset", y_offset);
        telemetry.update();
        sleep(1000);
        Position next = new Position();
        next.x =0;
        next.y= 0;
        moveto(next);
    }

    private void kickLeft(boolean isLeft)
    {
        if (isLeft) {
            telemetry.addLine("kick left");
            encoderDrive(TURN_SPEED,   -5, 5, 4.0);
            jewelHitter.setPosition(0.05);
            sleep(200);
            encoderDrive(TURN_SPEED,   5, -5, 4.0);
        } else {
            telemetry.addLine("kick right");
            encoderDrive(TURN_SPEED,   5, -5, 4.0);
            jewelHitter.setPosition(0.05);
            sleep(200);

            encoderDrive(TURN_SPEED,   -5, 5, 4.0);
        }
    }

    private int  redAtRight() {
        sleep(100);
        if (colorRange.red() > colorRange.blue() && colorRange.red()>=100) {
            telemetry.addLine("red at right");
            sleep(20);
            if (colorRange.red() > colorRange.blue()) {
                return 1;
            }
        } else if (colorRange.red() < colorRange.blue() && colorRange.blue()>=100){
            telemetry.addLine("red at left");
            return 0;
        }
        return -1;
    }

    private void kickOpponentJewel(boolean teamRed) {
         /* Detect the color */
        if (teamRed) {
            telemetry.addLine("Team red");
            /* kick blue */
            if (redAtRight() == 1) {
                kickLeft(true);
            } else if (redAtRight() == 0) {
                kickLeft(false);
            } else {
                jewelHitter.setPosition(0.05);
            }
        } else { /*kick red */
            telemetry.addLine("Team blue");
            if (redAtRight() == 1) {
                kickLeft(false);
            } else if(redAtRight() == 0) {
                kickLeft(true);
            } else {
                jewelHitter.setPosition(0.05);
            }
        }
        telemetry.update();
    }


    private void updateMyPitLocation() {
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark= RelicRecoveryVuMark.UNKNOWN;
        RelicRecoveryVuMark vuMark1;
        int offset = 0;
        telemetry.addLine("Scanning");
        telemetry.update();

        while (vuMark == RelicRecoveryVuMark.UNKNOWN) {

            if (offset < -12 ) {
                myPictoLocation = 0;
                telemetry.addData("can't figure out VuMark", offset);
                telemetry.update();
                break;
            }
            encoderDrive(TURN_SPEED,   -4, 4, 4.0);
            sleep(200);
            offset = offset - 4;
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        telemetry.addData("VuMark offset", offset );
        telemetry.update();
        vuMark1 =  RelicRecoveryVuMark.from(relicTemplate);

        while (vuMark != vuMark1) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            vuMark1 =  RelicRecoveryVuMark.from(relicTemplate);
        }

        if (vuMark==RelicRecoveryVuMark.LEFT){
            myPictoLocation = 1;
            telemetry.addData("get position left", myPictoLocation );
        } else if (vuMark==RelicRecoveryVuMark.CENTER){
            myPictoLocation = 0;
            telemetry.addData("get position center", myPictoLocation );
        } else if (vuMark==RelicRecoveryVuMark.RIGHT) {
            myPictoLocation = -1;
            telemetry.addData("get position center", myPictoLocation );
        }
        telemetry.update();
        encoderDrive(TURN_SPEED,   - offset, offset, 4.0);
        telemetry.addData("VuMark position", myPictoLocation );
        telemetry.update();
    }

    private void getOffAdjust() {
        if (myBSPosition == 1) {
            drivetime(1.0,1.0,1.0,1.0,1500);
            imudrive(90,0.5);
        } else if (myBSPosition == 2) {
            drivetime(-1.0,-1.0,-1.0,-1.0,1500);
        } else if (myBSPosition == 3) {
            drivetime(-1.0,-1.0,-1.0,-1.0,1500);
            imudrive(90,0.5);
        } else {
            encoderDrive(DRIVE_SPEED, -24, -24, 4.0);
        }
    }


    private void adjustScoreAngel() {
        if (myBSPosition == 1 ) {
            imudrive(-90,0.3);
        } else if(myBSPosition == 2) {
            imudrive(45,0.3);

        }  else if(myBSPosition == 3) {
            imudrive(90,0.3);
        } else {
            imudrive(45,0.3);
        }
    }

    private double LocationOffset() {
        double offset = 0;

        if ( myBSPosition == 1 || myBSPosition == 2 ) {
            offset = 7.6;
        } else if ( myBSPosition == 3 || myBSPosition == 4) {
            offset = - 7.6 ;
        }

        if (myPictoLocation == -1 ) {
            offset =  0 - offset;
        } else if(myPictoLocation == 0) {
            offset = 0;
        }
        telemetry.addData("LocationOffset", offset);
        telemetry.update();
        return offset;

    }

    private void scorePositioning() {

         /*drive to cryptobox */
        double offset = LocationOffset();
        double to_center;
        double distance = 0;

        if (myBSPosition == 1 || myBSPosition == 3) {
            to_center = 12;
            if (myBSPosition == 1) {
                distance = (to_center + offset);
            } else if (myBSPosition == 3) {
                distance = -(to_center + offset);
            }
            encoderDrive(DRIVE_SPEED,36,36,4);
        }
        if (myBSPosition == 2 || myBSPosition == 4) {
            to_center = 36;
            if (myBSPosition == 2) {
                distance = (to_center + offset);
            } else if (myBSPosition == 4) {
                distance = -(to_center + offset);
            }
            encoderDrive(DRIVE_SPEED, distance, distance, 4.0);
            imudrive(-90, 0.17);

        }
    }
    private void scoreGlyphs() {
        encoderDrive(DRIVE_SPEED,12,12,2);
        sleep(200);
        encoderDrive(DRIVE_SPEED,-3.5,-3.5,1.5);
        glyphDumper.setPower(-1.0);
        sleep(100);
        glyphDumper.setPower(1.0);
        sleep(1000);
        glyphDumper.setPower(0);
    }
    private void imudistance(double distance, double drivespeed) {

    }
}
