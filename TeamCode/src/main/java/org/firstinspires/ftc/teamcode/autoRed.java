package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.ftccommon.DbgLog;
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

@TeleOp(name = "BigBoyRed" , group = "Autonomous")
public class autoRed extends OpMode{
    //varibles
    int robo = 0;
    long time;
    int index = 0;

    ArrayList<Byte> ranges = new ArrayList<Byte>();

    ArrayList<Byte> temp = new ArrayList<Byte>();

    //motors
    DcMotor motorRightFront, motorRightBack, motorLeftFront, motorLeftBack;

    // ColorSensor color;
    CRServo servo;
    CRServo servo2;
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

    byte[] colorAcache;
    byte[] colorCcache;

    I2cDevice colorA;
    I2cDevice colorC;
    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorCreader;

    TouchSensor Spongebob;         //Instance of TouchSensor - for changing color sensor mode

    boolean touchState = false;  //Tracks the last known state of the touch sensor
    boolean LEDState = true;     //Tracks the mode of the color sensor; Active = true, Passive = false

    boolean done = false;
    boolean adjust = false;
    boolean direction = false;

    @Override
    public void init() {

        motorRightFront = hardwareMap.dcMotor.get("rightfront");
        motorRightBack = hardwareMap.dcMotor.get("rightback");
        motorLeftFront = hardwareMap.dcMotor.get("leftfront");
        motorLeftBack = hardwareMap.dcMotor.get("leftback");
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        ODSright = hardwareMap.opticalDistanceSensor.get("odsright");
        ODSleft = hardwareMap.opticalDistanceSensor.get("odsleft");
        //color = hardwareMap.colorSensor.get("color");
        servo = hardwareMap.crservo.get("servo");
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        servo2 = hardwareMap.crservo.get("servo2");

        colorA = hardwareMap.i2cDevice.get("cc");
        colorC = hardwareMap.i2cDevice.get("ca");

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x42), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x70), false);

        colorAreader.engage();
        colorCreader.engage();

        Spongebob = hardwareMap.touchSensor.get("t");
        colorCreader.write8(3, 1);

    }
    public void start(){
        colorAreader.write8(3, 1);
        colorCreader.write8(3, 1);
    }
    double color = 0;
    double color2 = 0;
    double count = 1;
    double initRange = 0;
    public void loop() {

        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        colorCcache = colorCreader.read(0x04, 1);

        telemetry.addData("2 #C", colorCcache[0] & 0xFF);
        color = colorCcache[0] & 0xFF;
        double rightTrim = 0, leftTrim = 0;
        beaconSet();
        switch (robo) {
            case 0:


                setThrottle(0.5);
                initRange = 0;
                robo++;
                break;
            case 1:
                if(count==2 || count==4){
                    if (initRange == 0) initRange = range1Cache[0];
                    if (initRange>23) initRange = 23;
                    if (initRange<10) initRange = 15;

                    if (!adjust) wallFollow(initRange);
                    if (adjust) adjust();
                }
                if(ODSright.getRawLightDetected() > 1.21){
                    setThrottle(-0.1);
                    robo++;
                }

                break;
            case 2:
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
                motorRightBack.setPower(-rightTrim);
                motorRightFront.setPower(-rightTrim);
                motorLeftBack.setPower(-leftTrim);
                motorLeftFront.setPower(-leftTrim);

                break;
            case 3:
                if(ODSleft.getRawLightDetected() < 1.51 || ODSright.getRawLightDetected() < 1.51){
                    servo.setPower(0);
                    setThrottle(-0.1);
                    robo--;
                    break;
                }
                servo.setPower(-1);
                if(color>=10 && color2>=10){
                    servo.setPower(1);
                    robo++;
                }
                break;
            case 4:
                robo++;
                break;
            case 5:
                robo++;
                break;
            case 6:
                if(Spongebob.isPressed()){
                    servo.setPower(0);
                    robo++;
                    if(count==4){
                        robo = -1;
                        setThrottle(0);
                        break;
                    }
                    setThrottle(0.3);
                    time = System.currentTimeMillis();
                }
                break;
            case 7:
                if(System.currentTimeMillis() - time >= 850){
                    if(count == 1 || count == 3){
                        robo = 0;
                        count++;
                    } else if(count == 2){
                        robo++;
                        count++;
                    } else{
                        robo = -1;
                        setThrottle(0);
                    }
                }
                break;
            case 8:
                //
                motorRightBack.setPower(0);
                motorRightFront.setPower(0);
                motorLeftBack.setPower(0.3);
                motorLeftFront.setPower(0.3);
                robo++;
                ranges.clear();
                index = 0;
                break;
            case 9:
                ranges.add(range1Cache[0]);

                if (ranges.size() == 1)
                    break;
                index++;
                if(Math.abs(range1Cache[0] - ranges.get(index-1))>100 || range1Cache[0] < 10){
                    index--;
                    ranges.remove(ranges.size()-1);
                    DbgLog.msg("Removed Bad Data " + range1Cache);
                    break;
                }
                DbgLog.msg("Case 9:" + range1Cache[0] +"," + (ranges.get(index) - ranges.get(index - 1)));
                if (ranges.get(index) - ranges.get(index - 1) <= -2) {
                    robo++;
                    motorLeftBack.setPower(0.15);
                    motorLeftFront.setPower(0.15);
                    temp.addAll(ranges);
                    temp.add(null);
                    ranges.clear();
                    index = 0;

                }
                break;
            case 10:
                ranges.add(range1Cache[0]);

                if (ranges.size() == 1)
                    break;
                index++;
                if(Math.abs(range1Cache[0] - ranges.get(index-1))>100 || range1Cache[0] < 10){
                    index--;
                    ranges.remove(ranges.size()-1);
                    DbgLog.msg("Removed Bad Data " + range1Cache);
                    break;
                }
                DbgLog.msg("Case 10:" + range1Cache[0] +"," + (ranges.get(index) - ranges.get(index - 1)));
                if (ranges.get(index) - ranges.get(index - 1) >= 1) {
                    setThrottle(0);
                    temp.addAll(ranges);
                    robo=0;


                }
                break;


        }



        telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
        telemetry.addData("ODS", range1Cache[1] & 0xFF);
//        telemetry.addData("Blue", color.blue());
//        telemetry.addData("RED", color.red());
//        telemetry.addData("Green", color.green());
        telemetry.addData("case", robo);
        telemetry.addData("count", count);
    }
    @Override
    public void stop(){

        DbgLog.msg(temp.toString());
        servo.setPower(0);
        servo2.setPower(0);
        setThrottle(0);
    }
    public void beaconSet(){
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

        if(color>=2 && color<=5 && color2>=2 && color2<=5){
            servo2.setPower(1);
        }
        telemetry.addData("input", color + ","+ color2);
    }
    public void setThrottle(double throttle){
        motorRightBack.setPower(throttle);
        motorRightFront.setPower(throttle);
        motorLeftBack.setPower(throttle);
        motorLeftFront.setPower(throttle);

    }
    public void wallFollow(double d){
        if(d-range1Cache[0] >= 2){
            motorRightBack.setPower(0);
            motorRightFront.setPower(0);
            motorLeftBack.setPower(0.1);
            motorLeftFront.setPower(0.1);
            direction = true;
            adjust = true;
            ranges.clear();
            index = 0;
        }
        if(d-range1Cache[0] <= -2){
            motorRightBack.setPower(0.1);
            motorRightFront.setPower(0.1);
            motorLeftBack.setPower(0);
            motorLeftFront.setPower(0);
            direction = false;
            adjust = true;
            ranges.clear();
            index = 0;
        }
    }

    public void adjust(){
        ranges.add(range1Cache[0]);

        if (ranges.size() == 1)
            return;
        index++;
        if(Math.abs(range1Cache[0] - ranges.get(index-1))>100 || range1Cache[0] < 5){
            index--;
            ranges.remove(ranges.size()-1);
            DbgLog.msg("Removed Bad Data " + range1Cache);
            return;
        }
        DbgLog.msg("Wall" + range1Cache[0] +"," + (ranges.get(index) - ranges.get(index - 1)));
        if(direction){
            if (ranges.get(index) - ranges.get(index - 1) >= 1) {
                setThrottle(0.3);
                adjust = false;
                ranges.clear();
            }
        }else{
            if (ranges.get(index) - ranges.get(index - 1) <= -1) {
                setThrottle(0.3);
                adjust = false;
                ranges.clear();
            }
        }

    }
}
