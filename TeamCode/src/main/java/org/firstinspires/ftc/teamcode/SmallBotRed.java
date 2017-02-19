package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import java.util.ArrayList;

@TeleOp(name = "SmallBotManual" , group = "Manual")
public class SmallBotRed extends OpMode{
    //varibles
    int robo = 1;
    long time;
    int index = 0;

    ArrayList<Byte> ranges = new ArrayList<Byte>();

    //motors
    DcMotor left;
    DcMotor right;

    ColorSensor color;
    CRServo servo;
    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;
    OpticalDistanceSensor ODS;
    //
    @Override
    public void init() {

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        ODS = hardwareMap.opticalDistanceSensor.get("odsright");
        color = hardwareMap.colorSensor.get("color");
        servo = hardwareMap.crservo.get("servo");
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

    }
    //
    public void loop(){

        switch (robo){
            case 1:
                setThrottle(.3);
                if (ODS.getRawLightDetected() > 1.51 ){
                    left.setPower(.1);
                    right.setPower(-.1);
                    robo++;
                }
                break;
            case 2:

                ranges.add(range1Cache[0]);
                if (ranges.size() == 1)
                    break;
                index++;
                if (ranges.get(index) - ranges.get(index - 1) >= 1){
                    setThrottle(0);
                    servo.setPower(-0.75);
                    robo++;

                }


                break;
            case 3:
                if(color.red() > 2){
                    robo++;
                    break;
                }
                if(color.blue()>1){
                    servo.setPower(0);
                    robo++;
                    setThrottle(-0.1);
                }
                break;

            case 4:
                if(color.red()>1){
                    setThrottle(0);
                    servo.setPower(-0.75);
                    robo++;
                }
                break;
            case 5:
                if(color.red()<1){
                    servo.setPower(1);
                    time = System.currentTimeMillis();
                    robo++;
                }
                break;
            case 6:
                if(System.currentTimeMillis() - time >-3000){
                    servo.setPower(0);
                }


        }




        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

        telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
        telemetry.addData("ODS", ODS.getRawLightDetected());
        //telemetry.addData("Color", );
        telemetry.addData("case: ", robo);
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
