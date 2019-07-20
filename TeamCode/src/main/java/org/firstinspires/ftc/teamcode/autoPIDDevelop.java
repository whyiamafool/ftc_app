package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous (name = "Autonomous PID Develop", group = "Autos")
@Disabled
public class autoPIDDevelop extends LinearOpMode {
    SummerHardware robot = new SummerHardware();
    public ElapsedTime runtime = new ElapsedTime();

    Orientation angles;

    BNO055IMU imu;

    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters             = new BNO055IMU.Parameters();
        parameters.angleUnit                        = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit                        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile              = "BNO055IMUCalibration.json";
        parameters.loggingEnabled                   = true;
        parameters.loggingTag                       = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            idle();
        }

        telemetry.addData("IMU calibraation status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status","Initialized");
        telemetry.update();

        waitForStart();
    }

    // solely for moving forward or backward
    // base method; will be enhanced later on
    public void pidDrive(int leftEncoder, int rightEncoder, double kP, double kI, double kD) {
        // resetting the runtime for accuracy
        runtime.reset();

        // setting target positions
        int leftTarget  = ((robot.frontLeft.getCurrentPosition() + robot.rearLeft.getCurrentPosition()) / 2) + leftEncoder;
        int rightTarget = ((robot.frontRight.getCurrentPosition() + robot.rearRight.getCurrentPosition()) / 2) + rightEncoder;

        // setting errors
        double leftError = leftEncoder;
        double rightError = rightEncoder;

        double totalLeftError = 0;
        double totalRightError = 0;

        // setting prevTime to time before loop
        double prevTime = getRuntime();
        while (Math.abs(leftError) != 0 && Math.abs(rightError) != 0) { // not at target yet
            double currTime = getRuntime();
            double deltaTime = currTime - prevTime;

            double currLeftPos = (robot.frontLeft.getCurrentPosition() + robot.rearLeft.getCurrentPosition()) / 2;
            double currRightPos = (robot.frontRight.getCurrentPosition() + robot.rearRight.getCurrentPosition()) / 2;

            leftError = leftTarget - currLeftPos;
            rightError = rightTarget - currRightPos;

            // defining the total errors
            totalLeftError += leftError*deltaTime;
            totalRightError += rightError*deltaTime;

            // setting the proportional values
            double pLeft = kP * leftError;
            double pRight = kP * rightError;

            // setting the integral values
            double iLeft = kI * totalLeftError;
            double iRight = kI * totalRightError;

            // setting the derivative values
            double dLeft = kD * (leftError/deltaTime);
            double dRight = kD * (rightError/deltaTime);

            // setting the motor powers to the sum of p, i, and d
            double leftMotorPower = pLeft + iLeft + dLeft;
            double rightMotorPower = pRight + iRight + dRight;

            // setting the motor powers
            robot.frontLeft.setPower(leftMotorPower);
            robot.frontRight.setPower(rightMotorPower);
            robot.rearLeft.setPower(leftMotorPower);
            robot.rearRight.setPower(rightMotorPower);

            prevTime = currTime;
        }
    }

    // used for tank turns
    // this is the base method; will be enhanced later on
    public void pidTurn(int turnAmount, int kP, int kI, int kD, double endHeading) {
        runtime.reset();

        int leftTarget = turnAmount + ((robot.frontLeft.getCurrentPosition() + robot.rearLeft.getCurrentPosition()) / 2);
        int rightTarget = turnAmount - ((robot.frontRight.getCurrentPosition() + robot.rearRight.getCurrentPosition()) / 2);

        double leftError = turnAmount;
        double rightError = turnAmount;

        double totalLeftError = 0;
        double totalRightError = 0;
        double totalHeadingError = 0;

        double prevHeading = angles.firstAngle;
        double headingError = endHeading - prevHeading;

        double prevTime = getRuntime();

        while (Math.abs(headingError) != 0 || (Math.abs(leftError) != 0 && Math.abs(rightError) != 0)) {
            double heading = angles.firstAngle;

            double currTime = getRuntime();
            double deltaTime = currTime - prevTime;

            double currLeftPos = (robot.frontLeft.getCurrentPosition() + robot.rearLeft.getCurrentPosition()) / 2;
            double currRightPos = (robot.frontRight.getCurrentPosition() + robot.rearRight.getCurrentPosition()) / 2;

            leftError = leftTarget - currLeftPos;
            rightError = rightTarget - currRightPos;

            headingError = endHeading - heading;

            // defining the total errors
            totalLeftError += leftError*deltaTime;
            totalRightError += rightError*deltaTime;
            totalHeadingError += headingError*deltaTime;

            // setting the proportional values
            double pLeft = ((-kP * leftError) + (-kP * headingError)) / 2;
            double pRight = ((kP * rightError) + (kP * headingError)) / 2;

            // setting the integral values
            double iLeft = ((-kI * totalLeftError) + (-kI * totalHeadingError)) / 2;
            double iRight = ((kI * totalRightError) + (kI * totalHeadingError)) / 2;

            // setting the derivative values
            double dLeft = ((-kD * (leftError/deltaTime)) + (-kD * (headingError/deltaTime))) / 2;
            double dRight = ((kD * (rightError/deltaTime)) + (kD * (headingError/deltaTime))) / 2;

            // setting the motor powers to the sum of p, i, and d
            double leftMotorPower = pLeft + iLeft + dLeft;
            double rightMotorPower = pRight + iRight + dRight;

            // setting the motor powers
            robot.frontLeft.setPower(leftMotorPower);
            robot.frontRight.setPower(rightMotorPower);
            robot.rearLeft.setPower(leftMotorPower);
            robot.rearRight.setPower(rightMotorPower);

            prevTime = currTime;
        }
    }

    // allows the robot to move in any direction by strafing or by simply moving forward and backward
    // still a work in progress
    public void pidMove(double moveInches, double kP, double kI, double kD, float angle) {
    }
}