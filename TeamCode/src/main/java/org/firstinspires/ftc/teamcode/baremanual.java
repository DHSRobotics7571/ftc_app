package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Andre on 12/18/2016.
 */
public class baremanual extends drivetraintest {
    DcMotor motorFeeder;
    DcMotor motorCatapult;
    @Override
    public void init(){
        super.init();
        motorFeeder = hardwareMap.dcMotor.get("feeder");

        motorCatapult = hardwareMap.dcMotor.get("catapult");

    }
    @Override
    public void loop(){
        super.loop();
        motorFeeder.setPower(gamepad2.left_stick_y);
        motorCatapult.setPower(gamepad2.right_stick_y);
    }
}
