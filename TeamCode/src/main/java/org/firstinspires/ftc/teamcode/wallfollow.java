package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp()
public class wallfollow extends OpMode {
    DcMotor mLF,mLB,mRF,mRB;
    OpticalDistanceSensor ODS;

    static double odsReadingRaw;
    static double odsReadingLinear;
    @Override
    public void init(){
        mRF = hardwareMap.dcMotor.get("rightfront");
        mRB = hardwareMap.dcMotor.get("rightback");
        mLF = hardwareMap.dcMotor.get("leftfront");
        mLB = hardwareMap.dcMotor.get("leftback");
        mLF.setDirection(DcMotorSimple.Direction.REVERSE);
        mLB.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop(){
        odsReadingRaw = ODS.getRawLightDetected();
        odsReadingLinear = Math.pow(odsReadingRaw, -0.5);

        mRB.setPower(odsReadingLinear * 2);
        mRF.setPower(odsReadingLinear * 2);
        mLB.setPower(0.5 - (odsReadingLinear * 2));
        mLF.setPower(0.5 - (odsReadingLinear * 2));
    }

    @Override
    public void stop(){
        mRB.setPower(0);
        mRF.setPower(0);
        mLB.setPower(0);
        mLF.setPower(0);
    }

}
