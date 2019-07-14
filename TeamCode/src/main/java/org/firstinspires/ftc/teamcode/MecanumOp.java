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
        
        double lFrontPow = gamepad1.left_stick_y - (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
        double lRearPow  = gamepad1.left_stick_y + (gamepad1.left_stick_x) - -gamepad1.right_stick_x;
        double rFrontPow = gamepad1.left_stick_y + (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
        double rRearPow  = gamepad1.left_stick_y - (gamepad1.left_stick_x) - -gamepad1.right_stick_x;

        double[] wheelPows = {lFrontPow,lRearPow,rFrontPow,rRearPow};

        Arrays.sort(wheelPows);

        if (wheelPows[3] > 1) {
            lFrontPow /= wheelPows[3];
            lRearPow /= wheelPows[3];
            rFrontPow /= wheelPows[3];
            rRearPow /= wheelPows[3];
        }

        // set motor powers
        robot.frontLeft.setPower(lFrontPow);
        robot.frontRight.setPower(rFrontPow);
        robot.rearLeft.setPower(lRearPow);
        robot.rearRight.setPower(rRearPow);
    }
}
