package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Manual", group="Manual")
public class manual extends OpMode {
    //instantiate objects
    private DcMotor motorRightFront,motorRightBack,motorLeftFront,motorLeftBack;
    @Override
    public void init(){
        motorRightFront = hardwareMap.dcMotor.get("rightfront");
        motorRightBack = hardwareMap.dcMotor.get("rightback");
        motorLeftFront = hardwareMap.dcMotor.get("leftfront");
        motorLeftBack = hardwareMap.dcMotor.get("leftback");
    }
    @Override
    public void loop(){
        //Mecanum wheel drive
        motorRightFront.setPower(-gamepad1.left_stick_x);

    }
    @Override
    public void stop(){

    }

}