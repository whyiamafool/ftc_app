package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
//@Disabled
public class DemoOp extends LinearOpMode {

    DemoHardware robot = new DemoHardware();
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

        boolean slowMode = true;

        while (opModeIsActive()) {

            //gamepad 1 (logitech)
            double throttle = -gamepad1.left_stick_y;
            double steering = gamepad1.left_stick_x/2;

            double rPower = throttle - steering;
            double lPower = throttle + steering;

            if (slowMode == true) {
                if (steering != 0) {
                    lPower /= 2.5;
                    rPower /= 2.5;
                } else {
                    lPower /= 2;
                    rPower /= 2;
                }
            }

            //gamepad 1 (logitech) setPower
            robot.rearLeft.setPower(lPower/4);
            robot.rearRight.setPower(rPower/4);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
