package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Andre on 12/18/2016.
 */
@TeleOp(name="Drive + Feeder/catapult", group="Manual")
@Disabled
public class baremanual extends drivetraintest {
    DcMotor motorFeeder;
    DcMotor motorCatapult;
    DcMotor motorLR;
    DcMotor motorLL;
    @Override
    public void init(){
        super.init();
        motorFeeder = hardwareMap.dcMotor.get("feeder");

        motorCatapult = hardwareMap.dcMotor.get("catapult");

        motorLR = hardwareMap.dcMotor.get("linearr");
        motorLL = hardwareMap.dcMotor.get("linearl");

    }
    @Override
    public void loop(){
        super.loop();
        motorFeeder.setPower(gamepad2.left_stick_y);
        motorCatapult.setPower(gamepad2.right_stick_y);
        motorLL.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
        motorLR.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
    }
}
