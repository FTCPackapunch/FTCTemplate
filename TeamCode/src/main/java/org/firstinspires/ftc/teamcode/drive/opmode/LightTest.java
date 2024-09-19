package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class LightTest extends LinearOpMode {
Servo LEDLight;

    @Override
    public void runOpMode() {
        // Get the LED colors and touch sensor from the hardwaremap
      LEDLight  = hardwareMap.get(Servo.class, "LED");


        // Wait for the play button to be pressed
        LEDLight.setPosition(0.8);
        waitForStart();

        // change LED mode from input to output

        // Loop while the Op Mode is running
        while (opModeIsActive()) {
            if(gamepad1.y){//White
              LEDLight.setPosition(0.76);
            }
            if(gamepad1.x){//purple
               LEDLight.setPosition(0.75);

            }
            if(gamepad1.b){//dark blue
                LEDLight.setPosition(0.74);

            }
            if(gamepad1.a){//light blue
                LEDLight.setPosition(0.73);

            }
            if(gamepad1.dpad_left){//even lighter blue
                LEDLight.setPosition(0.72);

            }
            if(gamepad1.dpad_up){//green
                LEDLight.setPosition(0.71);

            }
            if(gamepad1.dpad_right){//yellow green
                LEDLight.setPosition(0.7);

            }
            if(gamepad1.dpad_down){//even yellow green
                LEDLight.setPosition(0.69);

            }
            if(gamepad1.left_bumper){//yellow
                LEDLight.setPosition(0.68);

            }
            if(gamepad1.right_bumper){//red
                LEDLight.setPosition(0.67);

            }
            if(gamepad2.right_bumper){//pink
                LEDLight.setPosition(0.66);

            }    if(gamepad2.left_bumper){//weird pattren
                LEDLight.setPosition(0.65);

            }
        }

        }
    }
