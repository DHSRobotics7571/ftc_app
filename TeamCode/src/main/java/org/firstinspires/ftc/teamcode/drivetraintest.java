package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name="Manual", group="Manual")
public class drivetraintest extends OpMode {
    //instantiate objects
    private DcMotor motorRightFront,motorRightBack,motorLeftFront,motorLeftBack;

    @Override
    public void init(){
        motorRightFront = hardwareMap.dcMotor.get("rightfront");
        motorRightBack = hardwareMap.dcMotor.get("rightback");
        motorLeftFront = hardwareMap.dcMotor.get("leftfront");
        motorLeftBack = hardwareMap.dcMotor.get("leftback");
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
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
            up                                      up
            down                                    down
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


        //Mecanum wheel drive - Vector Addition and subtraction
        float[] motorVals = {0,0,0,0};
        //Right Front
        motorVals[0] = -gamepad1.left_stick_y-gamepad1.right_stick_x-gamepad1.left_stick_x;
        //Right Back
        motorVals[1] = -gamepad1.left_stick_y-gamepad1.right_stick_x+gamepad1.left_stick_x;
        //Left Front
        motorVals[2] = -gamepad1.left_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_x;
        //Left Back
        motorVals[3] = -gamepad1.left_stick_y+gamepad1.right_stick_x-gamepad1.left_stick_x;
        //Adjust range to that allowed by DcMotors
        motorVals = map(motorVals,-1,1);
        //Set power to motors
        motorRightFront.setPower(motorVals[0]);
        motorRightBack.setPower(motorVals[1]);
        motorLeftFront.setPower(motorVals[2]);
        motorLeftBack.setPower(motorVals[3]);
    }
    @Override
    public void stop(){

    }
    //you dont need to know what this does.
    public static float[] map(float[] a, double minR, double maxR){
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