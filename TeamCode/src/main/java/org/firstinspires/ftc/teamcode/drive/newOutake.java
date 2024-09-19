/* Copyright (c) 2023 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@TeleOp(name="newOutake Teleop", group = "Concept")
@Disabled
public class newOutake extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private DcMotor leftLinearSlide   = null;  //  Used to control the left front drive wheel
    private DcMotor rightLinearSlide  = null;  //  Used to control the right front drive wheel
    private DcMotor intakePrimary   = null;  //  Used to control the left front drive wheel
    private DcMotor intakeSecondary  = null;  //  Used to control the right front drive wheel




    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 0;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    static final double bucketIntakePosition    =  0.9;


    static final double bucketHalfPosition     =  0.80;

    static final double bucketDropfPosition     =  0.38;

    static final double armIntakePosition     =  0.62;

    static final double armOuttakePosition     =  0.56;
    //.58
    static final double armOuttakeHighPosition     =  0.60;

    static final double armOuttakeMorePosition     =  0.62;



    static final double     COUNTS_PER_MOTOR_REV    = 537.7;


    Servo   bucketServo;
    Servo mainServo;
    Servo droneTilt;
    Servo droneRelease;
    Servo drone;


    boolean rampUp = true;
    boolean manualLift = true;

   double ticks = 537.7;
   double newTarget;
    @Override public void runOpMode() throws InterruptedException {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;
        double  lift           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)


        // Initialize the Apriltag Detection process


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftLinearSlide = hardwareMap.get(DcMotorEx.class, "leftLinear");
        rightLinearSlide = hardwareMap.get(DcMotorEx.class, "rightLinear");
        intakePrimary = hardwareMap.get(DcMotorEx.class, "intakeP");
        intakeSecondary = hardwareMap.get(DcMotorEx.class, "intakeS");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        mainServo = hardwareMap.get(Servo.class, "mainServo");
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         droneTilt= hardwareMap.get(Servo.class, "planeTilt");
        droneRelease= hardwareMap.get(Servo.class, "planeRelease");




        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        intakePrimary.setDirection(DcMotor.Direction.FORWARD);
        intakeSecondary.setDirection(DcMotor.Direction.FORWARD);



        waitForStart();
        while (opModeIsActive())
        {
            drive  = -gamepad1.left_stick_y ; // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x ;  // Reduce strafe rate to 50%.
            turn   = -gamepad1.right_stick_x ;  // Reduce turn rate to 33%.

            if (gamepad2.y){
                bucketServo.setPosition(bucketHalfPosition);
                mainServo.setPosition(armOuttakeHighPosition);
                intakeSecondary.setPower(0);
                intakePrimary.setPower(0);
                lineTwo(1);
            }

            if (gamepad2.left_bumper){
                resetLinearEncoder();
            }

            if (gamepad2.b){
                bucketServo.setPosition(bucketIntakePosition);
                mainServo.setPosition(armIntakePosition);
                intakePrimary.setPower(1);
                intakeSecondary.setPower(1);
                bottom(1);
            }



            if(gamepad2.a){

                bucketServo.setPosition(bucketHalfPosition);
                mainServo.setPosition(armOuttakePosition);
                intakeSecondary.setPower(0);
                intakePrimary.setPower(0);
                lineOne(1);


            }
if(gamepad2.right_bumper){
    mainServo.setPosition(armOuttakeMorePosition);
}

            if(gamepad2.dpad_down){

                intakePrimary.setPower(-1);
                intakeSecondary.setPower(-1);

            }
            if(gamepad2.dpad_up){

              intakePrimary.setPower(0);
              intakeSecondary.setPower(0);
                bucketServo.setPosition(bucketHalfPosition);

            }

            if(gamepad2.x){

               bucketServo.setPosition(bucketDropfPosition);

            }



            if(gamepad1.dpad_up){
                hangUp(1);
            }
            if(gamepad1.dpad_down){
                hangDown(1);
            }
            //drone tilt close servo = 0.79
            if(gamepad1.left_bumper){
                droneRelease.setPosition(0.99);

            }
            if(gamepad1.right_bumper){
            droneRelease.setPosition(0.74);

        }
            if(gamepad1.a){


            }


            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);





        }






    }

    private void hangUp(int turnage) {
        newTarget = ticks;
        rightLinearSlide.setTargetPosition(1988);
        rightLinearSlide.setPower(1);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setTargetPosition(1982);
        leftLinearSlide.setPower(1);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void hangDown(int turnage){
        newTarget = ticks;
        rightLinearSlide.setTargetPosition(886);
        rightLinearSlide.setPower(0.5);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setTargetPosition(875);
        leftLinearSlide.setPower(0.5);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void lineOne(int turnage){
        newTarget = ticks;
        rightLinearSlide.setTargetPosition(2202);
        rightLinearSlide.setPower(1);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setTargetPosition(2290);
        leftLinearSlide.setPower(1);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void lineTwo(int turnage){
        newTarget = ticks;
        rightLinearSlide.setTargetPosition(2831);
        rightLinearSlide.setPower(1);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setTargetPosition(2843);
        leftLinearSlide.setPower(1);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void bottom(int turnage){
        newTarget = ticks;
        rightLinearSlide.setTargetPosition(0);
        rightLinearSlide.setPower(1);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setTargetPosition(0);
        leftLinearSlide.setPower(1);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetLinearEncoder(){

        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;



        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));



        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;

        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);





    }

    public void moveLift(double x) {
        // Calculate wheel powers.
        double leftPower    =  x ;
        double rightPower   =  x;




        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));


        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        // Send powers to the wheels.
       // leftLinearSlide.setPower(leftPower);
       // rightLinearSlide.setPower(-rightPower);




    }
/*
    public void InLineEncoderDriveLinearSlide_DBL(DcMotorEx mainMotor, DcMotorEx followerMotor , double speed, double slidemovement, double timeoutS) {
        int newArmTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newArmTarget = mainMotor.getCurrentPosition() + (int) (slidemovement * COUNTS_PER_INCH);
            mainMotor.setPower(Math.abs(speed));
            followerMotor.setPower(Math.abs(speed));
            mainMotor.setTargetPosition(newArmTarget);
            followerMotor.setTargetPosition(newArmTarget);

            // Turn On RUN_TO_POSITION
            mainMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            followerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && mainMotor.isBusy() && followerMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1", "LeftMotor Running to %7d ", newArmTarget);
                telemetry.addData("Path2", "Left Motor Running at %7d ", mainMotor.getCurrentPosition());
                telemetry.addData("Path1", "Right Motor Running to %7d ", newArmTarget);
                telemetry.addData("Path2", "Right Motor Running at %7d ", followerMotor.getCurrentPosition());
                telemetry.update();
                //sleep(5000);
            }

            sleep(1);   // optional pause after each move
        }

    }

 */

}



    /**
     * Initialize the AprilTag processor.
     */


    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
