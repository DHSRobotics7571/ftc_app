package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Andre on 12/18/2016.
 */
@TeleOp(name="ArcadeDriveTrainTest", group="Manual")
public class arcadeDrivetraintest extends drivetraintest {

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
        motorRightFront.setPower(-gamepad1.right_stick_y);
        motorRightBack.setPower(-gamepad1.right_stick_y);
        motorLeftFront.setPower(-gamepad1.left_stick_y);
        motorLeftBack.setPower(-gamepad1.left_stick_y);

        if(gamepad1.right_bumper)
            strafe(1);
        if(gamepad1.left_bumper)
            strafe(-1);


    }
    public void strafe(double power){
        motorRightBack.setPower(power);
        motorRightFront.setPower(-power);
        motorLeftBack.setPower(power);
        motorLeftFront.setPower(-power);
    }
}
