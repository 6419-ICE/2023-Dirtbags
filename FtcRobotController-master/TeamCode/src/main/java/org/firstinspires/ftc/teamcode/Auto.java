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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.PipelineBlue;
import org.firstinspires.ftc.teamcode.pipelines.PipelineRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Disabled
@Autonomous(name="Auto", group="Robot")
public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
    public OpenCvCamera webcam;
    PipelineBlue pipelineBlue;
    PipelineRed pipelineRed;
    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;
    private DcMotor         intake      = null;
    private Servo           servo       = null;
    private ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 200 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * 8/3) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     LEFT_CORRECTION         = 0.9;
    static final double     RIGHT_CORRECTION        = 1.0;
    static final double     TURN_SPEED              = 0.5;
    int color = 1; //1 = Blue, -1 = Red
    public Auto(int color)
    {
        super();
        this.color= color;
    }
    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        intake     = hardwareMap.get(DcMotor.class, "intake");
        servo      = hardwareMap.get(Servo.class, "servo");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                          leftDrive.getCurrentPosition(),
                          rightDrive.getCurrentPosition());
        telemetry.update();



        //while (!opModeIsActive()) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipelineBlue = new PipelineBlue();
        pipelineRed = new PipelineRed();

        if (color == 1) {
            webcam.setPipeline(pipelineBlue);
        } else {
            webcam.setPipeline(pipelineRed);
        }

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        double ElementZone = 0;
        while (!opModeIsActive()) {
            if (color == 1) {
                ElementZone = pipelineBlue.get_element_zone();
            } else {
                ElementZone = pipelineRed.get_element_zone();
            }
            telemetry.addData("Current zone: " ,ElementZone);
            telemetry.update();
        }


        waitForStart();







        if (ElementZone == 1) {
            encoderDrive(DRIVE_SPEED,  27, 5.0);
            turnDegrees(TURN_SPEED, 45, 10);
            encoderDrive(DRIVE_SPEED, 6, 10);
            //OUTTAKE CODE HERE
            sleep(1000);
            encoderDrive(DRIVE_SPEED, -2, 10);
            turnDegrees(TURN_SPEED, 45, 10);
            encoderDrive(DRIVE_SPEED, 39, 10);

        } else if (ElementZone == 3) {
            encoderDrive(DRIVE_SPEED,  27, 5.0);
            turnDegrees(TURN_SPEED, -45, 10);
            encoderDrive(DRIVE_SPEED, 6, 10);
            //OUTTAKE CODE HERE
            sleep(1000);
            encoderDrive(DRIVE_SPEED, -2, 10);
            turnDegrees(TURN_SPEED, 90, 10);
            encoderDrive(DRIVE_SPEED, 20, 10);
            turnDegrees(TURN_SPEED, 45, 10);
            encoderDrive(DRIVE_SPEED, 30, 10);
        } else {
            encoderDrive(DRIVE_SPEED,  29, 5.0);
            sleep(2000);
            encoderDrive(DRIVE_SPEED, -2, 5);
            turnDegrees(TURN_SPEED, 65, 10);
            encoderDrive(DRIVE_SPEED, 20, 10);
            turnDegrees(TURN_SPEED, 35, 10);
            encoderDrive(DRIVE_SPEED, 25, 10);

        }



        servo.setPosition(0);
        sleep(3000);
        servo.setPosition(1);

       // encoderDrive(DRIVE_SPEED, -5, 10);

        //turnDegrees(TURN_SPEED, -color*90, 10);

        //encoderDrive(DRIVE_SPEED, -25, 10);

        //turnDegrees(TURN_SPEED, -color * 90, 10);

        //encoderDrive(DRIVE_SPEED, -10, 5);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double Inches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed * LEFT_CORRECTION));
            rightDrive.setPower(Math.abs(speed * RIGHT_CORRECTION));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }


    public void turnDegrees(double speed,
                             double degrees,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            double leftInchesDouble = degrees*-12/90;
            double rightInchesDouble = degrees*12/90;
            int leftInches = (int) leftInchesDouble;
            int rightInches = (int) rightInchesDouble;

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void Outtake(double speed, double milliseconds) {
        runtime.reset();
        while (runtime.milliseconds() < milliseconds) {
            intake.setPower(speed);
        }
        intake.setPower(0);
    }

    public void Intake (double speed, double milliseconds) {
        runtime.reset();
        while (runtime.milliseconds() < milliseconds) {
            intake.setPower(-speed);
        }
        intake.setPower(0);
    }
}
