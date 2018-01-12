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
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;

import java.io.IOException;

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
    private Servo glyphLifter;
    private LynxI2cColorRangeSensor colorRange;
    private BNO055IMU imu;
    private Orientation angles;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private BNO055IMU.AccelerationIntegrator myIntegrator;
    private Position startPosition = null;
    private Position targetPosition = null;
    private final boolean myTeamRed = true;
    private int myBSPosition = 1; /* 1: red 1 2: red 2  3: blue 1 4:blue 2*/
    private int myPictoLocation = 0;    /* 1 : left 0: center -1 : right */

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.6;
    static final double     MY_K1              = 0.17; // Jan 4th gary change MY_K1 = 0.17  1/2/2018 gary change 2 1/2/2018 MY_K1 = 0.19 to make robot go farther

    //------------------------------------------------------------------------------------------------
    // Construction
    //------------------------------------------------------------------------------------------------

    public void initialize () {
        try {
            Logging.setup();
            Logging.log("Start Logging");
        } catch (IOException e1) {

            telemetry.addLine("Init Logging failed");
        }
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
        glyphLifter = hardwareMap.get(Servo.class,"glyphLifter");
        drivelf.setDirection(DcMotor.Direction.REVERSE);
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
        imu.startAccelerationIntegration(new Position(), new Velocity(), 20);
        startPosition = clonePosition(imu.getPosition());
        Logging.log(" start position x= %.3f, y= %.3f", startPosition.x, startPosition.y);
        telemetry.addLine("Init Ready");
        telemetry.update();
        Logging.log("Init succeed!");
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.clear();
        initialize();
        /* start of the code */
        waitForStart();
        /* lower jewel hitter, wait until in position */

        jewelHitter.setPosition(0.6);

        while (jewelHitter.getPosition()!=0.6) {
            sleep(100);
            telemetry.addData("Servo Position",jewelHitter.getPosition());
            telemetry.update();
        }
        sleep(200);

        kickOpponentJewel(myTeamRed); */
        /* get my pit location by scan the Vumark */
        updateMyPitLocation();

        /* get to the right postion before unload Glyphs */
        scorePositioning();
        /* unloading */
        scoreGlyphs1();
        imu.stopAccelerationIntegration();
    }

    private Position clonePosition(Position a) {
        return new Position(a.unit,
                a.x, a.y, a.z,
                a.acquisitionTime);
    }
    private Position normalizePosition(Position a) {
        // imu start with 90 degree,  so need replace (x, y) with (y, -x)
        return new Position(a.unit,
                a.y, - a.x, a.z,
                a.acquisitionTime);

    }

    private  void encoderDrive(double speed,
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
        newLeftTarget = (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = (int)(rightInches * COUNTS_PER_INCH);
        Logging.log("Front wheel Running to %7d :%7d", newLeftTarget,  newRightTarget);

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

        while (opModeIsActive()&& (runtime.seconds() < timeoutS) &&
                (drivelf.isBusy() && driverf.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    drivelf.getCurrentPosition(),
                    driverf.getCurrentPosition());
            telemetry.update();
            Logging.log("Front wheel Running at %7d :%7d", drivelf.getCurrentPosition(), driverf.getCurrentPosition());
            Logging.log("back wheel Running at %7d :%7d", drivelb.getCurrentPosition(), driverb.getCurrentPosition());
        }

        // Stop all motion;
        drivelf.setPower(0);
        driverf.setPower(0);
        drivelb.setPower(0);
        driverb.setPower(0);

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
        Logging.log("drive distance, %.3f", currDegrees);
        Logging.log("drive distance, %.3f", degree_offset);
        while (opModeIsActive() && (Math.abs(degree_offset) > 0.6) ){
            if (++count >5)
                break;
            drive_distance  = (degree_offset * k1);
            Logging.log("drive distance, %.3f", drive_distance);
            telemetry.addData("drive distance", drive_distance);
            encoderDrive(TURN_SPEED, -drive_distance , drive_distance, 5.0);  // S2: Turn Right 12 Inches with 5Sec timeout
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currDegrees = revert_normalize(currDegrees, angles.firstAngle);
            degree_offset = tarDegrees - currDegrees;
            telemetry.addData("current degree", currDegrees);
            telemetry.addData("Degrees_offset", degree_offset);
            Logging.log("drive distance, %.3f", currDegrees);
            Logging.log("drive distance, %.3f", degree_offset);
            telemetry.update();
        }
    }

    private void drivetime(double lfPower,double rfPower, double lbPower, double rbPower,long milliseconds){
        drivelf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driverf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivelf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driverf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        imudrive(45, MY_K1);
        sleep(20);
        imudrive(90, MY_K1);
        sleep(20);
        imudrive(45, MY_K1);
        sleep(20);
        // imudrive(60, MY_K1);
        // sleep(1000);
        // imudrive(-120, MY_K1);
        // sleep(1000);
        // imudrive(-100, MY_K1);
    }



    private void moveto(Position destination) {
        Position current = normalizePosition(imu.getPosition());
        double x_offset = destination.x - current.x;
        double y_offset = destination.y - current.y;
        /* initial imu is on 90 degree position,  so we need switch x_offset with y_offse when
         * calculate turning degree.  whether to moving forward or moving backward depends on x,y
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
        imudrive(turn_degree, MY_K1);
        if (y_offset < 0) {
            distance = -distance;
        }

        while (Math.abs(distance) > 0.5) {
            imudrive(turn_degree, MY_K1);
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
        imudrive(90, MY_K1);
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
        telemetry.addLine("red " + colorRange.red() + " blue: " + colorRange.blue());
        if (colorRange.red() > colorRange.blue()) {
            telemetry.addLine("red at right");
            sleep(20);
            if (colorRange.red() > colorRange.blue()) {
                return 1;
            }
        } else if (colorRange.red() < colorRange.blue()){
            telemetry.addLine("red at left");
            return 0;
        }
        return -1;
    }

    private void kickOpponentJewel(boolean teamRed) {
         /* Detect the color */
        if (teamRed) {
            telemetry.addLine("0101 Team red");
            /* kick blue */
            final int rar = redAtRight();
            telemetry.addLine("color value: " + rar);
            if (rar == 1) {
                kickLeft(true);
            } else if (rar == 0) {
                kickLeft(false);
            } else {
                jewelHitter.setPosition(0.05);
            }
        } else { /*kick red */
            telemetry.addLine("0101 Team blue");
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

            if (offset < -6 ) {
                myPictoLocation = 0;
                telemetry.addData("can't figure out VuMark", offset);
                telemetry.update();
                break;
            }
            encoderDrive(TURN_SPEED,   -2, 2, 4.0);
            sleep(200);
            offset = offset - 2;
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
            telemetry.addData("get position right", myPictoLocation );
        }
        telemetry.update();
        encoderDrive(TURN_SPEED,   - offset, offset, 4.0);
        telemetry.addData("VuMark position", myPictoLocation );
        telemetry.update();
    }

    /*
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
    */

    /*
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
*/

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
        double distance_h = 0;
        double distance_v = 0;
        double bsoffset1 = 0.5;   /* forwarding direction */
        double bsoffset2 = 0.7;  /* jowler hit direction, make it longer?*/

        if (myBSPosition == 1 || myBSPosition == 3) {

            to_center = 12;
            distance_v = 24 + bsoffset1;
            if (myBSPosition == 1) {
                distance_h = (to_center + offset + bsoffset2);
                distance_v = 24 + bsoffset1;
                encoderDrive(DRIVE_SPEED, distance_v ,distance_v,4);
                imudrive(90,MY_K1);
                //encoderDrive(DRIVE_SPEED,12,12,4);
            } else {
                distance_h = -(to_center + offset + bsoffset2);
                distance_v = -(24 - bsoffset1);
                encoderDrive(DRIVE_SPEED, distance_v ,distance_v,4);
                imudrive(-90,MY_K1);
            }
            encoderDrive(DRIVE_SPEED, distance_h, distance_h,4);
            imudrive(-90,MY_K1);
        }
        if (myBSPosition == 2 || myBSPosition == 4) {
            to_center = 36;
            if (myBSPosition == 2) {
                distance_h = (to_center + offset + bsoffset1);
            } else {
                distance_h = -(to_center + offset - bsoffset1);
            }
            encoderDrive(DRIVE_SPEED, distance_h, distance_h, 4.0);

            imudrive(-90, MY_K1);
            encoderDrive(DRIVE_SPEED,-bsoffset2, -bsoffset2,1);
        }
    }

    private void scoreGlyphs() {
        encoderDrive(DRIVE_SPEED,12,12,2);
        sleep(200);
        encoderDrive(DRIVE_SPEED,-5,-5,1.5);
        intakeLeft.setPower(0.5);
        intakeRight.setPower(0.5);
        sleep(60);
        glyphLifter.setPosition(0.0);
        glyphDumper.setPower(-0.5);
        sleep(1000);
        glyphLifter.setPosition(0.5);
        glyphDumper.setPower(0.5);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        sleep(1000);
        glyphDumper.setPower(0);
        encoderDrive(DRIVE_SPEED,2,2,1.5);
        encoderDrive(DRIVE_SPEED,-3.5,-3.5,2);
    }

    private void scoreGlyphs1() {
        /* touch the door */
        encoderDrive(DRIVE_SPEED,12,12,2);
        sleep(200);
        /* leaving  enough space for drop the Glyph*/
        encoderDrive(DRIVE_SPEED,-6,-6,1.5);
        /* move Glyph out of convey belt for easy lifting */
        intakeLeft.setPower(0.5);
        intakeRight.setPower(0.5);
        sleep(300);
        /* start shooting, initially with bigger power then slow down */
        glyphLifter.setPosition(0.0);
        glyphDumper.setPower(-0.4);
        sleep(400);
        /* use less power to slow down, leaving enough time to make sure it reach to top */
        glyphDumper.setPower(-0.3);
        sleep(1000);
        /* finish shoot,  reset everything */
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        glyphLifter.setPosition(0.5);
        glyphDumper.setPower(0.5);
        sleep(800);
        glyphDumper.setPower(0);
        /* leave more room to drop glyph*/
        encoderDrive(DRIVE_SPEED,-2,-2,1.0);
        /* push it in */
        encoderDrive(DRIVE_SPEED,5,5,1.0);
        /* leave a space and stop */
        encoderDrive(DRIVE_SPEED,-3.5,-3.5,1.0);
    }
}