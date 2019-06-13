package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

            //gamepad 1 (xbox) setPower
            robot.frontLeft.setPower(lPower/4);
            robot.frontRight.setPower(rPower/4);
            robot.rearLeft.setPower(lPower/4);
            robot.rearRight.setPower(rPower/4);

            robot.pivot.setPower(pivotMovement/3);
            robot.slide.setPower(slideMovement);
            robot.intake.setPower(intake);

            telemetry.addData("intake", intake);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
