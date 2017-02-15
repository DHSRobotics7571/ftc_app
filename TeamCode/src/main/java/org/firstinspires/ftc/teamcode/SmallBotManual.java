package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

@TeleOp(name = "SmallBotManual" , group = "Manual")
public class SmallBotManual extends OpMode{
    //varibles
    int robo = 1;
    long time;

    //motors
    DcMotor left;
    DcMotor right;

    ColorSensor color;

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;

    //
    @Override
    public void init() {

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        color = hardwareMap.colorSensor.get("color");

        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

    }
    //
    public void loop(){


        right.setPower(gamepad1.right_stick_y);
        left.setPower(gamepad1.left_stick_y);




        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

        telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
        telemetry.addData("ODS", range1Cache[1] & 0xFF);
        //telemetry.addData("Color", );
    }
    public void setThrottle(double throttle){
        left.setPower(throttle);
        right.setPower(throttle);

    }
    public void wallFollow(){
        if (range1Cache[0] > 5) {
            right.setPower(.2);
            left.setPower(.4);
            time = System.currentTimeMillis();

        }
        if (range1Cache[0] <= 5){
            left.setPower(.2);
            right.setPower(.4);
        }
        if (range1Cache[0] <= 3){

            time = System.currentTimeMillis();

        }

        //reachBeacon
        robo++;
    }
}
