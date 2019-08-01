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

@Autonomous (name = "autoPIDDevelop", group = "autoTesting")
// @Disabled
public class autoPIDDevelop extends LinearOpMode {
    SummerHardware robot = new SummerHardware();
    private ElapsedTime runtime = new ElapsedTime();

    Orientation angles, correctAngles; // DO NOT use correctAngles unless you have Pranav's permission

    BNO055IMU imu;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        telemetry.addData("Status","Initialized");
        telemetry.update();

        waitForStart();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        pidDrive(1200,1200,0.01,0.003,0,2.5);
        //pidMove(1200,-1200,0.5,0.5,0,0,0,0,90);
    }

    // for moving forward or backward or for steering turns
    /**
     * @param leftEncoder  the value that the left side of the dt needs to move (REQUIRED > 0)
     * @param rightEncoder the value that the right side of the dt needs to move (REQUIRED > 0)
     * @param kP           the proportional input of speed to the robot (REQUIRED > 0)
     * @param kI           the integral input to increasingly decrease offset error
     * @param kD           the derivative input that increasingly decreases overshoot
     */
    public void pidDrive(int leftEncoder, int rightEncoder, double kP, double kI, double kD, double acceptRange) {
        robot.resetMotorEncoders();
        robot.runUsingEncoder();
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
        while ((Math.abs(leftError) != 0 && Math.abs(rightError) != 0) && opModeIsActive()) { // not at target yet
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
        telemetry.addData("fL currPos",robot.fL.getCurrentPosition());
        telemetry.addData("fR currPos",robot.fR.getCurrentPosition());
        telemetry.update();
        sleep(2500);
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
        robot.runUsingEncoder();
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
     * @param pMoveTicks      how many ticks the positive part of dt needs to move
     * @param nMoveTicks      how many ticks the negative part of dt needs to move
     * @param pkP             proportional value for positive part of dt
     * @param nkP             proportional value for negative part of dt
     * @param pkI             integral input for positive part of dt
     * @param nkI             integral input fot negative part of dt
     * @param pkD             derivative input for positive part of dt
     * @param nkD             derivative input for negative part of dt
     * @param angle           angle robot must move at
     */
    public void pidMove(int pMoveTicks, int nMoveTicks, double pkP, double nkP, double pkI, double nkI, double pkD, double nkD, float angle) {
        runtime.reset();
        robot.resetMotorEncoders();
        robot.runUsingEncoder();

        angles.firstAngle = angle;

        double pError       = pMoveTicks;
        double nError       = nMoveTicks;
        double headingError = angle - angles.firstAngle;

        double totalPosError     = 0;
        double totalNegError     = 0;
        double totalHeadingError = 0;

        double prevTime = getRuntime();

        double pMotorPower = 0;
        double nMotorPower = 0;

        while ((!(Math.abs(pError) != 0) && !(Math.abs(nError) != 0) && !(Math.abs(headingError) != 0)) && opModeIsActive()) {
            double currTime = getRuntime();
            double deltaTime = currTime - prevTime;
            prevTime = currTime;

            double pCurrPos = (robot.rL.getCurrentPosition() + robot.fR.getCurrentPosition()) / 2;
            double nCurrPos = (robot.fL.getCurrentPosition() + robot.rR.getCurrentPosition()) / 2;

            pError = pMoveTicks - pCurrPos;
            nError = nMoveTicks - nCurrPos;
            headingError = angle - angles.firstAngle;

            totalPosError += pError*deltaTime;
            totalNegError += nError*deltaTime;
            totalHeadingError += headingError*deltaTime;

            double pPos = ((pkP * pError) + (pkP * headingError))/2;
            double pNeg = ((nkP * nError) + (nkP * headingError))/2;

            double iPos = ((pkI * totalPosError) + (pkI * totalHeadingError))/2;
            double iNeg = ((nkI * totalNegError) + (nkI * totalHeadingError))/2;

            double dPos = ((pkD * (pError/deltaTime)) + (pkD * (headingError/deltaTime)))/2;
            double dNeg = ((nkD * (nError/deltaTime)) + (nkD * (headingError/deltaTime)))/2;

            pMotorPower = pPos + iPos + dPos;
            nMotorPower = pNeg + iNeg + dNeg;

            robot.fL.setPower(clip(nMotorPower));
            robot.fR.setPower(clip(pMotorPower));
            robot.rL.setPower(clip(pMotorPower));
            robot.rR.setPower(clip(nMotorPower));

            telemetry.addData("Current Pos Position", pCurrPos);
            telemetry.addData("Current Neg Position", nCurrPos);
            telemetry.addData("Pos Power",pMotorPower);
            telemetry.addData("Neg Power",nMotorPower);
            telemetry.update();
        }
        telemetry.addData("Path","Complete");
        telemetry.update();
        resetAngle();
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