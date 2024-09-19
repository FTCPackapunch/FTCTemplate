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

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot_TC
{
    /* Public OpMode members. */
    static public HardwareMap hardwareMap = null;
    static public DcMotorEx  leftFront = null;
    static public DcMotorEx  rightFront  = null;

    static public DcMotorEx  leftRear  = null;
    static public DcMotorEx  rightRear  = null;
    static public DcMotorEx leftLinearSlide  = null;  //  Used to control the left front drive wheel
    static public DcMotorEx rightLinearSlide = null; //  Used to control the right front drive wheel
    static public DcMotorEx intakePrimary   = null;  //  Used to control the left front drive wheel
    static public DcMotorEx intakeSecondary  = null;  //  Used to control the right front drive wheel
    static public CameraName Webcam = null;
    static public Servo bucketServo;
    static public Servo mainServo;
    static public Servo droneTilt;
    static public Servo droneRelease;
    static public Servo openServo;
    Servo LEDLight;

    static public Servo stackServo;
    static public Servo angleServo;
    double newTarget;
    static public Positions positionFinal;
    static public Rev2mDistanceSensor SensorRange;
    static public TouchSensor limit;
    static public final double     COUNTS_PER_MOTOR_REV    = 537.6;  // 1440;    // eg: TETRIX Motor Encoder
    static public final double     DRIVE_GEAR_REDUCTION    = 1 ;   // 1  // This is < 1.0 if geared UP
    static public final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static public final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    static public final double bucketIntakePosition    =  0.74;
    static public final double bucketDropfPosition     =  0.55;
    static public final double bucketHalfPosition     =  0.90;
    static public final double armIntakePosition     =  0.83;
    static public final double armOuttakePosition     =  0.54;
    //static public final double smallServoOpen     =  0.74;
    static public final double smallServoOpen     =  0.55;
    static public final double angleDown     =  0.51;
    static public final double angleUp     =  0.95;
    //static public final double smallServoClosed     =  0.39;
    static public final double smallServoClosed     =  0.15;
    static public final double stackServoHor     =  0.8;
    static public final double stackServoVer   =  0.95;
    int tem = 1;

    //stack servo values
//0.27 is stack
//0.18 is down
// 0.69 is up

    /* local OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private Telemetry telemetry;

    /* Initialize standard Hardware interfaces */
    public HardwarePushbot_TC(HardwareMap hwMap){
        init(hwMap);
    }
    public enum Positions {
        LEFT,
        MIDDLE,
        RIGHT;
    }

    private void init(HardwareMap hwMap) {
        // Define and Initialize Motors
        hardwareMap = hwMap;
        leftFront  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftRear =  hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");


        leftLinearSlide = hardwareMap.get(DcMotorEx.class, "leftLinear");
        rightLinearSlide = hardwareMap.get(DcMotorEx.class, "rightLinear");
        intakePrimary = hardwareMap.get(DcMotorEx.class, "intakeP");
        intakeSecondary = hardwareMap.get(DcMotorEx.class, "intakeS");

        //define our servo's used
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        mainServo = hardwareMap.get(Servo.class, "mainServo");
        droneTilt= hardwareMap.get(Servo.class, "planeTilt");
        droneRelease= hardwareMap.get(Servo.class, "planeRelease");
        openServo= hardwareMap.get(Servo.class, "openServo");
        stackServo = hardwareMap.get(Servo.class, "stackServo");
        angleServo = hardwareMap.get(Servo.class, "angleServo");
        LEDLight  = hardwareMap.get(Servo.class, "LED");
//o highest
        //0.22 drop
        //0.79 intake

        leftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and Initialize Motors
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);
   //     leftLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
     //   rightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        intakePrimary.setDirection(DcMotor.Direction.FORWARD);
        intakeSecondary.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftLinearSlide.setPower(0);
        rightLinearSlide.setPower(0);
        intakePrimary.setPower(0);
        intakeSecondary.setPower(0);

    }

    public void firstPosition(int turnage){
        newTarget = COUNTS_PER_MOTOR_REV;

        rightLinearSlide.setTargetPosition(-1587);
        rightLinearSlide.setPower(1);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setTargetPosition(1600);
        leftLinearSlide.setPower(1);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void slightUp(int turnage){
        newTarget = COUNTS_PER_MOTOR_REV;

        rightLinearSlide.setTargetPosition(-1757);
        rightLinearSlide.setPower(0.5);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setTargetPosition(1796);
        leftLinearSlide.setPower(0.5);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void fullDown(int turnage){
        newTarget = COUNTS_PER_MOTOR_REV;

        rightLinearSlide.setTargetPosition(0);
        rightLinearSlide.setPower(1);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setTargetPosition(0);
        leftLinearSlide.setPower(1);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void lineOne(int turnage){
        newTarget = COUNTS_PER_MOTOR_REV;
        rightLinearSlide.setTargetPosition(-1500);//-1935
        rightLinearSlide.setPower(1);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setTargetPosition(1531);//1971
        leftLinearSlide.setPower(1);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void lineTwo(int turnage){
        newTarget = COUNTS_PER_MOTOR_REV;
        rightLinearSlide.setTargetPosition(-2000);
        rightLinearSlide.setPower(1);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setTargetPosition(2075);
        leftLinearSlide.setPower(1);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void lineThree(int turnage){
        newTarget = COUNTS_PER_MOTOR_REV;
        rightLinearSlide.setTargetPosition(-2652);
        rightLinearSlide.setPower(1);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setTargetPosition(2677);
        leftLinearSlide.setPower(1);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }




    public void resetLinearEncoder(){

        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}

