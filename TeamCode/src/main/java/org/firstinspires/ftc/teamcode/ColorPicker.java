package org.firstinspires.ftc.teamcode;


import android.graphics.Color;
import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.SyncdDevice;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name = "BeaconPicker" , group = "Autonomous")
public class ColorPicker extends OpMode{
    CRServo servo2;
    CRServo servo;
    TouchSensor touch;
    byte[] colorAcache;
    byte[] colorCcache;

    I2cDevice colorA;
    I2cDevice colorC;
    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorCreader;

    @Override
    public void init() {
        servo2 = hardwareMap.crservo.get("servo2");
        servo = hardwareMap.crservo.get("servo");
        touch = hardwareMap.touchSensor.get("t");
        colorA = hardwareMap.i2cDevice.get("cc");
        colorC = hardwareMap.i2cDevice.get("ca");

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x42), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x70), false);

        colorAreader.engage();
        colorCreader.engage();




    }
    public void start(){
        colorAreader.write8(3, 1);
        colorCreader.write8(3, 1);
    }
    double throttle = 1;
    double color = 0;
    double color2 = 0;
    public void loop() {
        colorAcache = colorAreader.read(0x04, 1);
        colorCcache = colorCreader.read(0x04, 1);

        color = colorAcache[0] & 0xFF;
        color2 = colorCcache[0] & 0xFF;

        if(color2 >= 10){
            servo2.setPower(1);
        } else if(color>= 10){
            servo2.setPower(-1);
        } else{
            servo2.setPower(0);
        }

        if(touch.isPressed()){
            throttle*=-1;
        }
        servo.setPower(throttle);
        telemetry.addData("input", color + ","+ color2);
    }
}
