/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;
import java.util.Locale;

@Autonomous(name="autoIMU", group="autoTesting")
//@Disabled
public class autoIMU extends LinearOpMode {

    /* Declare OpMode members. */
    SummerHardware robot   = new SummerHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public String pitch;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.resetMotorEncoders();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        mecanumDrive(0.1,320); //1320
        mecanumDrive(0.1,-320);
        imuTurn(0.175,30);
        imuTurn(0.175,60);


        telemetry.addData("current", angles.firstAngle);
        telemetry.update();
        sleep(2000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void mecanumDrive (double speed, double encoder) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            sleep(750);

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.fL.getCurrentPosition() - (int)(encoder);// * COUNTS_PER_INCH);
            newFrontRightTarget = robot.fR.getCurrentPosition() - (int)(encoder);// * COUNTS_PER_INCH);
            newRearLeftTarget = robot.rL.getCurrentPosition() - (int)(encoder);// * COUNTS_PER_INCH);
            newRearRightTarget = robot.rR.getCurrentPosition() - (int)(encoder);// * COUNTS_PER_INCH);
            robot.fL.setTargetPosition(newFrontLeftTarget);
            robot.fR.setTargetPosition(newFrontRightTarget);
            robot.rL.setTargetPosition(newRearLeftTarget);
            robot.rR.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            robot.fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.fL.setPower(Math.abs(speed));
            robot.fR.setPower(Math.abs(speed));
            robot.rL.setPower(Math.abs(speed));
            robot.rR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.fL.isBusy() || robot.rL.isBusy() || robot.fR.isBusy() || robot.rR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.fL.getCurrentPosition(),
                        robot.fR.getCurrentPosition());
                telemetry.update();
            }

        }
    }

    public void mecanumTurn (double speed, double encoder) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            sleep(750);

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.fL.getCurrentPosition() - (int)(encoder);// * COUNTS_PER_INCH);
            newFrontRightTarget = robot.fR.getCurrentPosition() + (int)(encoder);// * COUNTS_PER_INCH);
            newRearLeftTarget = robot.rL.getCurrentPosition() - (int)(encoder);// * COUNTS_PER_INCH);
            newRearRightTarget = robot.rR.getCurrentPosition() + (int)(encoder);// * COUNTS_PER_INCH);
            robot.fL.setTargetPosition(newFrontLeftTarget);
            robot.fR.setTargetPosition(newFrontRightTarget);
            robot.rL.setTargetPosition(newRearLeftTarget);
            robot.rR.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            robot.fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.fL.setPower(Math.abs(speed));
            robot.fR.setPower(Math.abs(speed));
            robot.rL.setPower(Math.abs(speed));
            robot.rR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.fL.isBusy() || robot.rL.isBusy() || robot.fR.isBusy() || robot.rR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.fL.getCurrentPosition(),
                        robot.fR.getCurrentPosition());
                telemetry.update();
            }

        }
    }

    public void imuTurn (double speed, double bearing) {
        robot.resetMotorEncoders();
        robot.runUsingEncoder();

        double preHeading = angles.firstAngle;
        double currentHeading = 0;
        double deltaHeading = 1;
        double deltaRatio = 1;
        double ratioSpeed;

        boolean turnDone = false;
        boolean timerStart = false;
        while(!turnDone && opModeIsActive()) {
            currentHeading = angles.firstAngle - preHeading;
            deltaHeading = bearing - currentHeading;
            deltaRatio = deltaHeading/bearing;
            if (bearing > 0) {
                ratioSpeed = speed * deltaRatio;
                if (deltaRatio < 0 && ratioSpeed > -0.015) {
                    ratioSpeed = -0.005;
                } else if (deltaRatio > 0 && ratioSpeed < 0.015) {
                    ratioSpeed = 0.005;
                }
                if (Math.abs(deltaHeading) < 0.5) {
                    if (!timerStart) {
                        runtime.reset();
                        timerStart = true;
                    } else if (runtime.seconds() > 0.1) {
                        turnDone = true;
                    }
                    ratioSpeed = 0;
                } else {
                    timerStart = false;
                }
                robot.fL.setPower(ratioSpeed);
                robot.fR.setPower(-ratioSpeed);
                robot.rL.setPower(ratioSpeed);
                robot.rR.setPower(-ratioSpeed);
                telemetry.addData("delta", deltaHeading);
                telemetry.addData("current", currentHeading);
                telemetry.addData("speed", ratioSpeed);
                telemetry.update();
            }
        }
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        /*telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        */

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        pitch = formatAngle(angles.angleUnit, angles.thirdAngle);
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        /*telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });

        telemetry.addLine()
                .addData("Debugging: ", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        */
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
