package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

@TeleOp
//@Disabled //DO NOT ENABLE UNLESS YOU HAVE ADI'S PERMISSION
public class SummerOp extends LinearOpMode {

    SummerHardware robot = new SummerHardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)

        boolean sorterOpen = false;

        while (opModeIsActive()) {

            //gamepad 1 (xbox)
            double throttle = -gamepad1.left_stick_y;
            double steering = gamepad1.left_stick_x/2;
            
            double rPower = throttle - steering;
            double lPower = throttle + steering;

            double pivotMovement = gamepad1.right_stick_y;
            double slideMovement = ((gamepad1.left_trigger) - (gamepad1.right_trigger));

            double intakeLeft = 0;
            double intakeRight = 0;

            if (gamepad1.left_bumper) {
                intakeLeft = 1;
            }
            if (gamepad1.right_bumper) {
                intakeRight = 1;
            }

            double intake = ((-intakeRight) + (intakeLeft));

            if (gamepad1.x) {
                intake = 0.4;
            }

            if (gamepad1.y) {
                robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (!sorterOpen && robot.slide.getCurrentPosition() >= 2000 && robot.pivot.getCurrentPosition() >= 1650) {
                sorterOpen = true;
                robot.gate.setPosition(1); //open
            } else if (sorterOpen && robot.slide.getCurrentPosition() <= 1700 && robot.pivot.getCurrentPosition() <= 950) {
                sorterOpen = false;
                robot.gate.setPosition(0.5); //close
            }

            //gamepad 1 (xbox) setPower
            robot.frontLeft.setPower(lPower/4);
            robot.frontRight.setPower(rPower/4);
            robot.rearLeft.setPower(lPower/4);
            robot.rearRight.setPower(rPower/4);

            robot.pivot.setPower(pivotMovement/3);
            robot.slide.setPower(slideMovement);
            robot.sLeft.setPower(-slideMovement);
            robot.intake.setPower(intake);

            telemetry.addData("intake", intake);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
