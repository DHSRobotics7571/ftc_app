package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Autonomous - BLUE", group="Autonomous")
public class autoBlue extends OpMode {
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


    }
    @Override
    public void stop(){

    }

}