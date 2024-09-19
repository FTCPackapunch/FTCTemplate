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
public class LightTime extends LinearOpMode {
    Servo LEDLight;

    @Override
    public void runOpMode() {
        // Get the LED colors and touch sensor from the hardwaremap
        LEDLight  = hardwareMap.get(Servo.class, "LED");

int tem = 1;
        LEDLight.setPosition(0.76);
        // Wait for the play button to be pressed
        waitForStart();

        // change LED mode from input to output

        // Loop while the Op Mode is running
        while (opModeIsActive()) {
            if (tem ==1){
               resetRuntime();
                tem = 2;
            }

            if (time >=10){
                LEDLight.setPosition(0.61);
            }




        }
    }
}