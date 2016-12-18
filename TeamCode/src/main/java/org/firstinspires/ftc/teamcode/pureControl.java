package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Andre on 12/18/2016.
 */
@TeleOp(name="Pure Control", group="Manual")
public class pureControl extends drivetraintest {
    @Override
    public void loop(){
        motorRightFront.setPower(gamepad1.left_stick_y);
        motorRightBack.setPower(gamepad1.right_stick_y);
        motorLeftFront.setPower(gamepad2.left_stick_y);
        motorLeftBack.setPower(gamepad2.right_stick_y);

    }
}
