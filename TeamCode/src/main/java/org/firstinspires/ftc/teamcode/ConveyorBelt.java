package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Test code for conveyor
 */
@TeleOp(name="Conveyor Test", group="Mecanum")
public class ConveyorBelt extends LinearOpMode {

    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    public ConveyorBelt() {
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
            } else {
                intakeRight.setPower(0);
                intakeLeft.setPower(0);
            }
        }
    }
}
