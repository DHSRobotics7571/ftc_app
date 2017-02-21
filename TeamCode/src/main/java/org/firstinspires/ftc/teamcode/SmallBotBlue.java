package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

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

@TeleOp(name = "SmallBotBlue - No Balls" , group = "Autonomous")
public class SmallBotBlue extends OpMode{
    //varibles
    int robo = 1;
    long time;
    int index = 0;

    ArrayList<Byte> ranges = new ArrayList<Byte>();

    //motors
    DcMotor left;
    DcMotor right;

    // ColorSensor color;
    CRServo servo;
    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;
    OpticalDistanceSensor ODSright;
    OpticalDistanceSensor ODSleft;
    //
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    byte[] colorCcache;

    I2cDevice colorC;
    I2cDeviceSynch colorCreader;

    TouchSensor touch;         //Instance of TouchSensor - for changing color sensor mode

    boolean touchState = false;  //Tracks the last known state of the touch sensor
    boolean LEDState = true;     //Tracks the mode of the color sensor; Active = true, Passive = false
    @Override
    public void init() {

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        ODSright = hardwareMap.opticalDistanceSensor.get("odsright");
        ODSleft = hardwareMap.opticalDistanceSensor.get("odsleft");
        //color = hardwareMap.colorSensor.get("color");
        servo = hardwareMap.crservo.get("servo");
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        telemetry.addData("Status", "Initialized");

        //the below lines set up the configuration file
        colorC = hardwareMap.i2cDevice.get("cc");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();

        touch = hardwareMap.touchSensor.get("t");
        colorCreader.write8(3, 1);

    }
    public void start(){
        colorCreader.write8(3, 1);
    }
    double color = 0;
    public void loop() {

        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        colorCcache = colorCreader.read(0x04, 1);

        telemetry.addData("2 #C", colorCcache[0] & 0xFF);
        color = colorCcache[0] & 0xFF;
        double rightTrim = 0, leftTrim = 0;
        switch (robo) {
            case 1:
                setThrottle(.3);
                if(ODSright.getRawLightDetected() > 1.51){
                    setThrottle(-0.1);
                    robo++;
                }

                break;
            case 2:
                //derivative();
                if(ODSleft.getRawLightDetected() > 1.51 && ODSright.getRawLightDetected() > 1.51){
                    servo.setPower(-0.3);
                    setThrottle(0);
                    robo++;
                    break;
                }
                if (ODSright.getRawLightDetected() > 1.51) {
                    leftTrim = -0.1;
                    rightTrim = 0.1;
                }else if(ODSleft.getRawLightDetected() > 1.51){
                    leftTrim = 0.1;
                    rightTrim = -0.1;
                }else{
                    rightTrim = 0.1; leftTrim= 0.1;
                }
                right.setPower(-rightTrim);
                left.setPower(-leftTrim);

                break;
            case 3:
                if(color >= 10){
                    servo.setPower(0);
                    setThrottle(-.05);
                    robo++;
                }
                if(color >= 1 && color <= 4){
                    robo++;
                }

                break;
            case 4:
                if(color >= 1 && color <= 4){
                    setThrottle(0);
                    servo.setPower(-1);
                    time = System.currentTimeMillis();
                    robo++;

                }
                break;
            case 5:
                if(System.currentTimeMillis() - time >= 3000){
                    robo++;
                    servo.setPower(1);
                    time = System.currentTimeMillis();
                }
                break;
            case 6:
                if(System.currentTimeMillis() - time >= 3000){
                    servo.setPower(0);
                    robo++;
                    setThrottle(0.3);
                    time = System.currentTimeMillis();
                }
                break;
            case 7:
                if(System.currentTimeMillis() - time >= 1000){
                    robo = 1;
                    break;
                }

        }



        telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
        telemetry.addData("ODS", range1Cache[1] & 0xFF);
//        telemetry.addData("Blue", color.blue());
//        telemetry.addData("RED", color.red());
//        telemetry.addData("Green", color.green());
        telemetry.addData("case", robo);
    }
    public void derivative(){
        ranges.add(range1Cache[0]);

        if (ranges.size() == 1)
            return;
        index++;
        if(Math.abs(range1Cache[0] - ranges.get(index-1))>100){
            index--;
            ranges.remove(ranges.size()-1);
            return;
        }

        if (ranges.get(index) - ranges.get(index - 1) >= 1) {
            setThrottle(0);
            servo.setPower(-0.75);
            time = System.currentTimeMillis();
            robo++;

        }
    }
    public void setThrottle(double throttle){
        left.setPower(throttle);
        right.setPower(throttle);

    }
    public void wallFollow(){
        double trim = 12-range1Cache[0];
        if(trim>0){
            trim*=trim;
        }else{
            trim*=-trim;
        }
        right.setPower(0.5-(0.01*(trim)));
        left.setPower(0.5+(0.01*(trim)));

        if(right.getPower()<0)right.setPower(0);
        if(left.getPower()<0)left.setPower(0);
    }
}
