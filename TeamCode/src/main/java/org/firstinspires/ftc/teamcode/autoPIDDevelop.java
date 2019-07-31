/*
* A program that contains all autonomous methods to be used
*
* @author Pranav Chitiveli
* @version 20190719
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous (name = "Autonomous PID Develop", group = "Autos")
// @Disabled
public class autoPIDDevelop extends LinearOpMode {
    SummerHardware robot = new SummerHardware();
    public ElapsedTime runtime = new ElapsedTime();

    Orientation angles, correctAngles = new Orientation(); // DO NOT use correctAngles unless you have Pranav's permission

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

    // for moving forward or backward or for steering turns
    /**
    * @param leftEncoder  the value that the left side of the dt needs to move (REQUIRED > 0)
    * @param rightEncoder the value that the right side of the dt needs to move (REQUIRED > 0)
    * @param kP           the proportional input of speed to the robot (REQUIRED > 0)
    * @param kI           the integral input to increasingly decrease offset error
    * @param kD           the derivative input that increasingly decreases overshoot
     */
    public void pidDrive(int leftEncoder, int rightEncoder, double kP, double kI, double kD) {
        robot.resetMotorEncoders();
        runtime.reset();

        // setting target positions
        int leftTarget  = ((robot.fL.getCurrentPosition() + robot.rL.getCurrentPosition()) / 2) + leftEncoder;
        int rightTarget = ((robot.fR.getCurrentPosition() + robot.rR.getCurrentPosition()) / 2) + rightEncoder;

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

            double currLeftPos = (robot.fL.getCurrentPosition() + robot.rL.getCurrentPosition()) / 2;
            double currRightPos = (robot.fR.getCurrentPosition() + robot.rR.getCurrentPosition()) / 2;

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
            robot.fL.setPower(clip(leftMotorPower));
            robot.fR.setPower(clip(rightMotorPower));
            robot.rL.setPower(clip(leftMotorPower));
            robot.rR.setPower(clip(rightMotorPower));

            prevTime = currTime;
        }
    }

    // used solely for tank turns
    /**
     * @param turnAmount the number of ticks (reversed on each side of dt) that the robot must turn (REQUIRED > 0)
     * @param kP         the proportional input of speed to the robot (REQUIRED > 0)
     * @param kI         the integral input to increasingly decrease offset error
     * @param kD         the derivative input that increasingly decreases overshoot
     */
    public void pidTurn(int turnAmount, int kP, int kI, int kD, double endHeading) {
        robot.resetMotorEncoders();
        runtime.reset();

        int leftTarget = turnAmount + ((robot.fL.getCurrentPosition() + robot.rL.getCurrentPosition()) / 2);
        int rightTarget = turnAmount - ((robot.fR.getCurrentPosition() + robot.rR.getCurrentPosition()) / 2);

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

            double currLeftPos = (robot.fL.getCurrentPosition() + robot.rL.getCurrentPosition()) / 2;
            double currRightPos = (robot.fR.getCurrentPosition() + robot.rR.getCurrentPosition()) / 2;

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
            robot.fL.setPower(clip(leftMotorPower));
            robot.fR.setPower(clip(rightMotorPower));
            robot.rL.setPower(clip(leftMotorPower));
            robot.rR.setPower(clip(rightMotorPower));

            prevTime = currTime;
        }
    }

    // allows the robot to move in any direction by strafing or by simply moving forward and backward
    // still a work in progress

    /**
     * @param pMoveTicks number of ticks the positive pair needs to move
     * @param nMoveTicks number of ticks the negative pair needs to move
     * @param kP         proportionally decreases speed as it approaches the target
     * @param kI         decreases steady-state error
     * @param kD         decreases probability of overshoot
     * @param angle
     */
    public void pidMove(int pMoveTicks, int nMoveTicks, double wheelErrorRange, double kP, double kI, double kD, float angle, double headingRange) {
        runtime.reset();
        robot.resetMotorEncoders();

        angles.firstAngle = angle;

        int pTar = pMoveTicks;
        int nTar = nMoveTicks;

        double pError       = pTar;
        double nError       = nTar;
        double headingError = angle - angles.firstAngle;

        double totalPosError     = 0;
        double totalNegError     = 0;
        double totalHeadingError = 0;

        double prevTime = getRuntime();

        while (!(Math.abs(pError) <= wheelErrorRange) && !(Math.abs(nError) <= wheelErrorRange) && !(Math.abs(headingError) <= headingRange)) {
            
        }
    }

    // resets the angle if firstAngle is changed
    void resetAngle() {
        angles.firstAngle = correctAngles.firstAngle;
    }

    /**
     * @param value a value that will be turned into a percentage
     * @return
     */
    double clip(double value) {
        return value / 100;
    }

    /**
     * @param ticks the number of encoder ticks to be converted into imperial inches
     * @return
     */
    double ticksToInches(double ticks) {
        double inches = 0; // 0 is placeholder value

        // code that converts encoder ticks to inches
        // work in progress

        return inches;
    }
}