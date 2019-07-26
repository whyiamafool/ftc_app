package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp
public class SDOp extends LinearOpMode {
    SummerHardware robot = new SummerHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status","Initialized");
        telemetry.update();

        waitForStart();

        boolean slowMode = false;

        while (opModeIsActive()) {
            double FrontLeftVal =  gamepad1.left_stick_y - (gamepad1.left_stick_x)  + -gamepad1.right_stick_x;
            double FrontRightVal =  gamepad1.left_stick_y  + (gamepad1.left_stick_x) - -gamepad1.right_stick_x;
            double BackLeftVal = gamepad1.left_stick_y  + (gamepad1.left_stick_x)  + -gamepad1.right_stick_x;
            double BackRightVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) - -gamepad1.right_stick_x;

            //Move range to between 0 and +1, if not already
            double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
            Arrays.sort(wheelPowers);
            if (wheelPowers[3] > 1) {
                FrontLeftVal /= wheelPowers[3];
                FrontRightVal /= wheelPowers[3];
                BackLeftVal /= wheelPowers[3];
                BackRightVal /= wheelPowers[3];
            }

            if (gamepad1.a) {
                slowMode = true;
            }
            if (gamepad1.b) {
                slowMode = false;
            }

            if (slowMode) {
                FrontLeftVal /= 2;
                FrontRightVal /= 2;
                BackLeftVal /= 2;
                BackRightVal /= 2;
            }

            robot.fL.setPower(FrontLeftVal);
            robot.fR.setPower(FrontRightVal);
            robot.rL.setPower(BackLeftVal);
            robot.rR.setPower(BackRightVal);

            telemetry.addData("FrontLeftPow",FrontLeftVal);
            telemetry.addData("FrontRightPow",FrontRightVal);
            telemetry.addData("RearLeftPow",BackLeftVal);
            telemetry.addData("RearRightPow",BackRightVal);
            telemetry.update();
        }
    }
}
