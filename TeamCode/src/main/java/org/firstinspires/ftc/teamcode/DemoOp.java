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
            double throttle = (gamepad1.right_trigger) - (gamepad1.left_trigger);
            double steering = -gamepad1.left_stick_x/2;

            double rPower = throttle - steering;
            double lPower = throttle + steering;

            if (throttle > 0) {
                if (steering > 0) {
                    lPower += steering;
                } else if (steering < 0) {
                    rPower += steering;
                }
            }

            //gamepad 1 (logitech) setPower
            robot.left_drive.setPower(lPower);
            robot.right_drive.setPower(rPower);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
