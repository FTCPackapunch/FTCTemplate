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

import static org.firstinspires.ftc.teamcode.Auto.HardwarePushbot_TC.stackServo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auto.HardwarePushbot_TC;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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

@TeleOp(name="TeleOp_CS", group = "Concept")
public class MecanumTeleOp_CS extends LinearOpMode
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
    static final double bucketIntakePosition    =  0.28;//0.74
    static final double bucketHalfPosition     =  0.51; //0.85
    static final double bucketDropfPosition     = 0.0 ;//0.61

    static final double armIntakePosition     =  0.79;
    static final double armOuttakePosition     =  0.35; //0 as highest
    static final double armHighOuttakePosition     =  0;
    static public final double smallServoOpen     =  0.50;
    static public final double smallServoClosed     =  0.12;
    static final double angleDown     =  0.49;
    static final double angleUp     =  0.95;
    //0.95 open
    //0.51 close
    Servo droneTilt;
    Servo droneRelease;
    Servo openServo;
    Servo LEDLight;

    double ticks = 537.7;
    double newTarget;
    HardwarePushbot_TC  robot;
   @Override
public void runOpMode() throws InterruptedException {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;
        double  lift           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        TrajectorySequence trajectoryParking;
        robot = new HardwarePushbot_TC(hardwareMap);
         // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
       leftLinearSlide = hardwareMap.get(DcMotorEx.class, "leftLinear");
       rightLinearSlide = hardwareMap.get(DcMotorEx.class, "rightLinear");
        robot.leftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        droneTilt= hardwareMap.get(Servo.class, "planeTilt");
        droneRelease= hardwareMap.get(Servo.class, "planeRelease");
        openServo= hardwareMap.get(Servo.class, "openServo");
       LEDLight  = hardwareMap.get(Servo.class, "LED");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        robot.leftFront.setDirection(DcMotor.Direction.REVERSE);
        robot.leftRear.setDirection(DcMotor.Direction.REVERSE);

        
        robot.rightFront.setDirection(DcMotor.Direction.FORWARD);
        robot.rightRear.setDirection(DcMotor.Direction.FORWARD);
        //robot.leftLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        //robot.rightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.intakePrimary.setDirection(DcMotor.Direction.FORWARD);
        robot.intakeSecondary.setDirection(DcMotor.Direction.FORWARD);

        SampleMecanumDrive drivex = new SampleMecanumDrive(hardwareMap);
        // Set your initial pose to x: 0, y: 0, facing 90 degrees
        drivex.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
       LEDLight.setPosition(0.76);

       int tem = 1;
        waitForStart();


        while (opModeIsActive())
        {

          int leftPos = leftLinearSlide.getCurrentPosition();
            int rightPos = rightLinearSlide.getCurrentPosition();


            drive  = -gamepad1.left_stick_y ; // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x ;  // Reduce strafe rate to 50%.
            turn   = -gamepad1.right_stick_x*0.65 ;  // Reduce turn rate to 33%.
            //All controls for Driver B
            //reset linear encoders for preset functionality
            if (gamepad2.left_bumper){
                resetLinearEncoder();
            }

            if (tem ==1){
                resetRuntime();
                tem = 2;
            }

            if (time >=90){
                LEDLight.setPosition(0.67);
            }
            if (time >=108){
                LEDLight.setPosition(0.71);
            }
            // reset all bucket and bucket arm to intake ready position
            if (gamepad2.dpad_down){
                robot.bucketServo.setPosition(bucketIntakePosition);
                robot.mainServo.setPosition(armIntakePosition);
                robot.intakePrimary.setPower(1);
                robot.intakeSecondary.setPower(1);
                robot.openServo.setPosition(smallServoClosed);
                robot.angleServo.setPosition(angleUp);
                //lower the linear slide to be base of the robot
                bottom(1);
            }

            //tilt bucket up and stop intake
            if(gamepad2.dpad_up){
                robot.intakePrimary.setPower(0);
                robot.intakeSecondary.setPower(0);
               // robot.bucketServo.setPosition(0.95);
                robot.angleServo.setPosition(angleDown);

            }
            //0.95 open
            //0.51 close

            //position to deliver at the level 1
            if(gamepad2.a){
                robot.bucketServo.setPosition(bucketDropfPosition);
                robot.mainServo.setPosition(armIntakePosition);
                robot.intakeSecondary.setPower(0);
                robot.intakePrimary.setPower(0);
                robot.lineOne(1);
                robot.mainServo.setPosition(armOuttakePosition);
            }
            //position to deliver at the level 2
            if (gamepad2.b){
                robot.bucketServo.setPosition(bucketDropfPosition);
                robot.mainServo.setPosition(armOuttakePosition);
                robot.intakeSecondary.setPower(0);
                robot.intakePrimary.setPower(0);
                lineTwo(1);
            }
            if (gamepad2.y){
                robot.bucketServo.setPosition(bucketDropfPosition);
                robot.mainServo.setPosition(armOuttakePosition);
                robot.intakeSecondary.setPower(0);
                robot.intakePrimary.setPower(0);
                lineThree(1);
            }

            //open the servo lock gate to release the pixels
            if (gamepad2.right_bumper){
                robot.openServo.setPosition(smallServoOpen);
            }
            if (gamepad2.x){
                robot.angleServo.setPosition(angleUp);
            }


            //close the pixel release lock
            if (gamepad2.dpad_right){
               robot.openServo.setPosition(smallServoClosed);
            }
            if (gamepad2.dpad_left){
                robot.bucketServo.setPosition(bucketHalfPosition);
            }



            //All controls for Driver A
            //reverse the intake motors
            if(gamepad1.a){
                robot.intakePrimary.setPower(-1);
                robot.intakeSecondary.setPower(-1);
            }

            if(gamepad1.y){
                robot.intakePrimary.setPower(0);
                robot.intakeSecondary.setPower(0);
            }

            if(gamepad1.x){
                robot.mainServo.setPosition(armIntakePosition);
            }

            if(gamepad1.y){
                robot.mainServo.setPosition(armOuttakePosition);
            }

            if(gamepad1.x){
                stackServo.setPosition(0.18);
            }

            if(gamepad1.dpad_up){
                hangUp(1);
            }
            if(gamepad2.right_trigger>0.5){
                robot.bucketServo.setPosition(bucketDropfPosition);
                robot.mainServo.setPosition(armHighOuttakePosition);
                robot.intakeSecondary.setPower(0);
                robot.intakePrimary.setPower(0);
                lineFour(1);
            }

            if(gamepad2.left_trigger>0.5){
             //   robot.intakeSecondary.setPower(0);
              //  robot.intakePrimary.setPower(0);
            leftPos += 200;
            rightPos -= 200;
                newTarget = ticks;
                robot.rightLinearSlide.setTargetPosition(rightPos);
                robot.rightLinearSlide.setPower(1);
                robot.rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftLinearSlide.setTargetPosition(leftPos);
                robot.leftLinearSlide.setPower(1);
                robot.leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if(gamepad1.dpad_down){
                hangDown(1);
            }
            //drone tilt close servo = 0.52
            //shoot the drone
            if(gamepad1.right_bumper){
                droneRelease.setPosition(0.61);
            }

            //lock the servo to hold the drone rubberband
            if(gamepad1.left_bumper) {
                droneRelease.setPosition(0.3);
            }

            drivex.update();
            Pose2d myPose = drivex.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());


            if (gamepad1.dpad_left) {
                // Insert whatever teleop code you're using
                //initPose = new Pose2d(0, 0, Math.toRadians(0));//Starting pose
                trajectoryParking = drivex.trajectorySequenceBuilder(myPose)
                        .strafeLeft(5)
                        .build();
                drivex.followTrajectorySequence(trajectoryParking);

            }
            if (gamepad1.dpad_right) {
                // Insert whatever teleop code you're using
                // initPose = new Pose2d(0, 0, Math.toRadians(0));//Starting pose
                trajectoryParking = drivex.trajectorySequenceBuilder(myPose)
                        .strafeRight(5)
                        .build();
                drivex.followTrajectorySequence(trajectoryParking);
            }
            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);



        }

    }









    private void hangUp(int turnage) {
        newTarget = ticks;
        robot.rightLinearSlide.setTargetPosition(-2188);
        robot.rightLinearSlide.setPower(1);
        robot.rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLinearSlide.setTargetPosition(2182);
        robot.leftLinearSlide.setPower(1);
        robot.leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void hangDown(int turnage){
        newTarget = ticks;
        robot.rightLinearSlide.setTargetPosition(-586);
        robot.rightLinearSlide.setPower(0.5);
        robot.rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLinearSlide.setTargetPosition(580);
        robot.leftLinearSlide.setPower(0.5);
        robot.leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void lineOne(int turnage){
        newTarget = ticks;
        robot.rightLinearSlide.setTargetPosition(-1945);
        robot.rightLinearSlide.setPower(1);
        robot.rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLinearSlide.setTargetPosition(1971);
        robot.leftLinearSlide.setPower(1);
        robot.leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void lineTwo(int turnage){
        newTarget = ticks;
        robot.rightLinearSlide.setTargetPosition(-2000);
        robot.rightLinearSlide.setPower(1);
        robot.rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLinearSlide.setTargetPosition(2006);
        robot.leftLinearSlide.setPower(1);
        robot.leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void lineThree(int turnage){
        newTarget = ticks;
        robot.rightLinearSlide.setTargetPosition(-2500);
        robot.rightLinearSlide.setPower(1);
        robot.rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLinearSlide.setTargetPosition(2525);
        robot.leftLinearSlide.setPower(1);
        robot.leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void lineFour(int turnage){
        newTarget = ticks;
        robot.rightLinearSlide.setTargetPosition(-2533);
        robot.rightLinearSlide.setPower(1);
        robot.rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLinearSlide.setTargetPosition(2549);
        robot.leftLinearSlide.setPower(1);
        robot.leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void bottom(int turnage){
        newTarget = ticks;
        robot.rightLinearSlide.setTargetPosition(0);
        robot.rightLinearSlide.setPower(1);
        robot.rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLinearSlide.setTargetPosition(0);
        robot.leftLinearSlide.setPower(1);
        robot.leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetLinearEncoder(){

        robot.rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        robot.leftFront.setPower(leftFrontPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.leftRear.setPower(leftBackPower);
        robot.rightRear.setPower(rightBackPower);





    }

}


