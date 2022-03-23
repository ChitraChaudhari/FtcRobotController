package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Gear ratio test")
public class GearRatioTest extends LinearOpMode {

    /* Declare OpMode members. */

    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.32 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.5;

    //creating and initializing four dc motor objects for all the drive motors on robot
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //hardware mapping dc motors to configuration files
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        //setting both the front right only motor in reverse direction
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addLine("Press Play to start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  24);  // S1: Forward 47 Inches with 5 Sec timeout


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double distanceInches) {
        int targetPosition;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            targetPosition = (int)(distanceInches * COUNTS_PER_INCH);

            backLeftMotor.setTargetPosition(targetPosition);
            backRightMotor.setTargetPosition(targetPosition);
            frontRightMotor.setTargetPosition(targetPosition);
            frontLeftMotor.setTargetPosition(targetPosition);

            // Turn On RUN_TO_POSITION
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            backLeftMotor.setPower(Math.abs(speed));
            backRightMotor.setPower(Math.abs(speed));
            frontRightMotor.setPower(Math.abs(speed));
            frontLeftMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (backRightMotor.isBusy() && backLeftMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy())) {
            }

            // Stop all motion;
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
