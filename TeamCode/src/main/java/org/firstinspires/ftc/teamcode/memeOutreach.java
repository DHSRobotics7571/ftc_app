package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.media.AudioTrack;
import android.media.MediaPlayer;
import android.media.MediaRecorder;
import android.provider.MediaStore;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.R.raw.friedNoodles;

@TeleOp(name="meme", group="Manual")
public class memeOutreach extends OpMode {
    //instantiate objects
    DcMotor motorRightFront,motorRightBack,motorLeftFront,motorLeftBack;
    DcMotor motorLinearRight,motorLinearLeft;
    DcMotor motorFeeder;
    DcMotor motorCatapult;

    Servo servoFeeder;
    Servo servoLinear;
    Servo servoLinear2;

    double throttlecontrol = 1;

    double power;

    MediaPlayer player;

    int count = 1;


    Context context;

    @Override
    public void init(){



        motorRightFront = hardwareMap.dcMotor.get("rightfront");
        motorRightBack = hardwareMap.dcMotor.get("rightback");
        motorLeftFront = hardwareMap.dcMotor.get("leftfront");
        motorLeftBack = hardwareMap.dcMotor.get("leftback");
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLinearRight = hardwareMap.dcMotor.get("linearr");
        motorLinearLeft = hardwareMap.dcMotor.get("linearl");

        motorFeeder = hardwareMap.dcMotor.get("feeder");

        motorCatapult = hardwareMap.dcMotor.get("catapult");

        servoFeeder = hardwareMap.servo.get("servofeeder");
        servoLinear = hardwareMap.servo.get("servolinear");
        servoLinear2 = hardwareMap.servo.get("servolinear2");


        //color = hardwareMap.colorSensor.get("color");

        servoLinear.setPosition(1);
        servoLinear2.setPosition(.5);

    }

    @Override
    public void loop(){
        /*
        Gamepad1's keymap                       Gamepad2's keymap
        DPAD                                    DPAD
            up                                      up
            down                                    down
            left                                    left
            right                                   right
        LEFT_STICK                              LEFT_STICK
            up                  -drive motor        up                  -ballfeed
            down                -drive motor        down                -ballfeed
            left                -drive motor        left
            right               -drive motor        right
        RIGHT_STICK                             RIGHT_STICK
            up                                     up                  -catapult
            down                                     down                -catapult
            left                -drive motor        left
            right               -drive motor        right
        BUTTONS                                 BUTTONS
            a                                       a
            x                                       x
            b                                       b
            y                                       y
        BUMPERS                                 BUMPERS
            left_trigger                            left_trigger        -linear motor
            left_bumper         -beacon             left_bumper
            right_trigger                           right_trigger       -linear motor
            right_bumper        -beacon             right_bumper

         */

        if(gamepad1.y){
            if (count == 1) {
                player = MediaPlayer.create(context, R.raw.friedNoodles);
                player.start();
            }
            if (count == 2) {
                player = MediaPlayer.create(context, R.raw.ultimate);
                player.start();
            }
            if (count == 3){
                player = MediaPlayer.create(context, R.raw.vsauce);
                player.start();
            }
        }

        if(gamepad1.x){
            if (count == 1) {
                player = MediaPlayer.create(context, R.raw.prettyGood);
                player.start();
            }
            if (count == 2) {
                player = MediaPlayer.create(context, R.raw.idubbzMedley);
                player.start();
            }
            if (count == 3){
                player = MediaPlayer.create(context, R.raw.friedNoodles);
                player.start();
            }
        }

        if((gamepad1.b) || (gamepad1.a)){
            player.stop();
        }

        if (gamepad1.dpad_up){
            count = count--;
        }

        if (gamepad1.dpad_left) {
            count = count++;
        }

        if(gamepad1.left_stick_button||gamepad1.right_stick_button){
            throttlecontrol = 0.35;
        }else throttlecontrol = 1;

        if (gamepad1.right_stick_button)
            servoFeeder.setPosition(.3);
            else
            servoFeeder.setPosition(0);

        //feeder
        if (gamepad1.right_bumper){
            motorFeeder.setPower(1);
        } else
            motorFeeder.setPower(0);

        //catapult
        if (gamepad1.right_bumper){
            motorCatapult.setPower(1);
        } else
            motorCatapult.setPower(0);



        //Mecanum wheel drive - Vector Addition and subtraction
        float[] motorVals = {0,0,0,0};
        //Right Front
        motorVals[0] = -gamepad1.left_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_x;
        //Right Back
        motorVals[1] = -gamepad1.left_stick_y+gamepad1.right_stick_x-gamepad1.left_stick_x;
        //Left Front
        motorVals[2] = -gamepad1.left_stick_y-gamepad1.right_stick_x-gamepad1.left_stick_x;
        //Left Back
        motorVals[3] = -gamepad1.left_stick_y-gamepad1.right_stick_x+gamepad1.left_stick_x;
        //Adjust range to that allowed by DcMotors
        motorVals = map(motorVals,-1,1);
        //Set power to motors
        motorRightFront.setPower(motorVals[0]*throttlecontrol);
        motorRightBack.setPower(motorVals[1]*throttlecontrol);
        motorLeftFront.setPower(motorVals[2]*throttlecontrol);
        motorLeftBack.setPower(motorVals[3]*throttlecontrol);

        motorLinearLeft.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        motorLinearRight.setPower(gamepad1.left_trigger - gamepad1.right_trigger);



    }

    @Override
    public void stop(){

    }
    //you dont need to know what this does
    public float[] map(float[] a, double minR, double maxR){
        float min=Float.MAX_VALUE;
        float max=Float.MIN_VALUE;
        for(int i = 0;i<=3;i++){
            if(a[i] < min){
                min = a[i];
            }
            if(a[i] > max){
                max = a[i];
            }
        }
        if(min>=-1&&max<=1){
            return a;
        }
        double scalemin = Math.abs(minR/min);
        double scalemax = Math.abs(minR/max);
        double finalscale;

        if(scalemin<scalemax)
            finalscale = scalemin;
        else
            finalscale = scalemax;
        for(int i = 0;i<=3;i++){
            a[i] = a[i]*(float)finalscale;
        }

        return a;
    }

}