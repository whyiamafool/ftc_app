package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp
public class MecanumOp extends LinearOpMode {
    SummerHardware robot = new SummerHardware();

    @Override
    public void runOpMode() {
        telemetry.addData("Status","Initialized");
        telemetry.update();

        waitForStart();

        double fLeftPow = gamepad1.left_stick_y - (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
        double rLeftPow  = gamepad1.left_stick_y + (gamepad1.left_stick_x) - -gamepad1.right_stick_x;
        double fRightPow = gamepad1.left_stick_y + (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
        double rRightPow  = gamepad1.left_stick_y - (gamepad1.left_stick_x) - -gamepad1.right_stick_x;

        double[] wheelPows = {fLeftPow,rLeftPow,fRightPow,rRightPow};

        Arrays.sort(wheelPows);

        if (wheelPows[3] > 1) {
            fLeftPow /= wheelPows[3];
            rLeftPow /= wheelPows[3];
            fRightPow /= wheelPows[3];
            rLeftPow /= wheelPows[3];
        }

        // set motor powers
        robot.frontLeft.setPower(fLeftPow);
        robot.frontRight.setPower(rLeftPow);
        robot.rearLeft.setPower(fRightPow);
        robot.rearRight.setPower(rRightPow);

        telemetry.addData("FrontLeftPow",fLeftPow);
        telemetry.addData("FrontRightPow",rRightPow);
        telemetry.addData("RearLeftPow",fRightPow);
        telemetry.addData("RearRightPow",rRightPow);
        telemetry.update();
    }
}
