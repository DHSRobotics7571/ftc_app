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
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SmallBot" , group = "Autonomous")
public class SmallBot extends OpMode{
    //varibles
    int robo = 0;
    long time;

    //motors
    DcMotor left;
    DcMotor right;

    CRServo pusher;

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
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        pusher = hardwareMap.crservo.get("servo");

        color = hardwareMap.colorSensor.get("color");

        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        //RANGE1Reader.engage();

    }
    //
    public void loop() {

        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        //right.setPower(0.3-(0.02*(20-range1Cache[0])));
        //left.setPower(0.3+(0.02*(20-range1Cache[0])));

        // if(right.getPower()<0)right.setPower(0);
        //if(left.getPower()<0)left.setPower(0);

        switch (robo) {
            case 0:
                setThrottle(.2);
                RANGE1Reader.engage();
                if (range1Cache[0] > 15) {
                    left.setPower(.1);
                    right.setPower(.3);
                }
                if (range1Cache[0] == 15){
                    setThrottle(.2);
                }

                if (range1Cache[0] < 15){
                    right.setPower(.1);
                    left.setPower(.3);
                }
                if (color.blue() < 3 && color.blue() > 3){
                    robo++;
                    break;
                }

            case 1:


                if (color.blue() < 3 && color.blue() > 1) {
                    setThrottle(0);
                    setThrottle(-.05);
                    if (color.blue() < 3 && color.blue() > 1){
                        setThrottle(0);
                        pusher.setPower(-.75);
                        time = System.currentTimeMillis();
                        robo++;
                        break;
                    }

                }
            case 2:
                if (System.currentTimeMillis() == time + 3000) {
                    pusher.setPower(1);
                    time = System.currentTimeMillis();
                    if (System.currentTimeMillis() == time + 3000) {
                        pusher.setPower(0);
                    }
                }


                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("ODS", range1Cache[1] & 0xFF);
                //telemetry.addData("Color", );
        }
    }
    public void setThrottle(double throttle){
        left.setPower(throttle);
        right.setPower(throttle);

    }
    public void wallFollow(){
        if (range1Cache[0] > 18) {
            left.setPower(.2);
            right.setPower(.4);
        }
        if (range1Cache[0] == 18){
            setThrottle(.2);
        }

        if (range1Cache[0] < 18){
            right.setPower(.2);
            left.setPower(.4);
        }

        //reachBeacon
        robo++;
    }
}
