package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
Functions to control a mecanum drive train using the umu.
 */
public class RHSGyroMecanum {
    static final double WHEEL_DIAMETER = 4;
    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    private DcMotorEx leftFront = null;
    private DcMotorEx rightRear = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private BNO055IMU imu = null;
    private imuDriveState currentDriveState;

    public RHSGyroMecanum(DcMotorEx leftFront, DcMotorEx rightRear, DcMotorEx leftRear,
                          DcMotorEx rightFront, BNO055IMU imu) {
        this.leftFront = leftFront;
        this.leftRear = leftRear;
        this.rightFront = rightFront;
        this.rightRear = rightRear;
        this.imu = imu;
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop and return true when Move gets to the desired position.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public boolean imuDrive(double speed,
                            double distance,
                            double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double velocity;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        switch (currentDriveState) {
            case IMU_DRIVE_STATE_START:
                // Determine new target position, and pass to motor controller
                moveCounts = getInchesToTicks(distance);
                newLeftTarget = leftFront.getCurrentPosition() + moveCounts;
                newRightTarget = rightFront.getCurrentPosition() + moveCounts;

                // Set Target and Turn On RUN_TO_POSITION
                leftFront.setTargetPosition(newLeftTarget);
                rightFront.setTargetPosition(newRightTarget);
                leftRear.setTargetPosition(newLeftTarget);
                rightRear.setTargetPosition(newRightTarget);

                leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                rightRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                velocity = getVelocityFromPower(speed);
                leftFront.setVelocity(velocity);
                rightFront.setVelocity(velocity);
                leftRear.setVelocity(velocity);
                rightRear.setVelocity(velocity);
                currentDriveState = imuDriveState.IMU_DRIVE_STATE_DRIVING;
                break;

            case IMU_DRIVE_STATE_DRIVING:
                // All motors are running.
                if (leftFront.isBusy()
                        && rightFront.isBusy()
                        && leftRear.isBusy()
                        && rightRear.isBusy()) {

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    // Adjust for setVelocity
                    leftSpeed *= getVelocityFromPower(leftSpeed);
                    rightSpeed += getVelocityFromPower(rightSpeed);

                    leftFront.setVelocity(leftSpeed);
                    rightFront.setVelocity(rightSpeed);
                    leftRear.setVelocity(leftSpeed);
                    rightRear.setVelocity(rightSpeed);
                    //updateTelemetry();
                } else {
                    currentDriveState = imuDriveState.IMU_DRIVE_STATE_STOPPED;
                }
                return false;

            case IMU_DRIVE_STATE_STOPPED:
                // Stop all motion;
                gyroStop();

                // Turn off RUN_TO_POSITION
                leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                return true;
            default:
                break;
        }

        return false;
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Returns true when heading is reached.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public boolean gyroTurn(double speed, double angle) {
        // keep looping while we are not on heading.
        return (onHeading(speed, angle, P_TURN_COEFF));
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while ((holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            //updateTelemetry();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        //Adjust for setVelocity
        leftSpeed = getVelocityFromPower(leftSpeed);
        rightSpeed = getVelocityFromPower(rightSpeed);

        // Send desired speeds to motors.
        leftFront.setVelocity(leftSpeed);
        rightFront.setVelocity(rightSpeed);
        leftRear.setVelocity(leftSpeed);
        rightRear.setVelocity(rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation().firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void gyroStop() {
        leftFront.setVelocity(0);
        rightFront.setVelocity(0);
        leftRear.setVelocity(0);
        rightRear.setVelocity(0);
    }

    private double getTicksPerSecond() {
        return (leftFront.getMotorType().getMaxRPM() / 60) * leftFront.getMotorType().getTicksPerRev();
    }

    private double getVelocityFromPower(double power) {
        return getTicksPerSecond() * power;
    }

    private int getInchesToTicks(double inches) {
        double circumference = WHEEL_DIAMETER * Math.PI;
        double rotations = inches / circumference;
        return (int) (rotations * leftFront.getMotorType().getTicksPerRev());
    }

    private enum imuDriveState {
        IMU_DRIVE_STATE_START,
        IMU_DRIVE_STATE_DRIVING,
        IMU_DRIVE_STATE_STOPPED
    }
}
