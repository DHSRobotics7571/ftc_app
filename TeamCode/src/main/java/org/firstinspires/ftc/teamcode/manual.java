package org.firstinspires.ftc.teamcode;

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

@TeleOp(name="Manual", group="Manual")
public class manual extends OpMode {
    //instantiate objects
    DcMotor motorRightFront,motorRightBack,motorLeftFront,motorLeftBack;
    DcMotor motorLinearRight,motorLinearLeft;
    DcMotor motorFeeder;
    DcMotor motorCatapult;

    CRServo servo;
    CRServo servo2;
    Servo servoFeeder;
    Servo servoLinear;
    Servo servoLinear2;
    double throttlecontrol = 1;
    OpticalDistanceSensor ODSright, ODSleft;
    double color = 0;
    double color2 = 0;

    byte[] colorAcache;
    byte[] colorCcache;

    I2cDevice colorA;
    I2cDevice colorC;
    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorCreader;


    //logic objects
    double power;
    @Override
    public void init(){
        motorRightFront = hardwareMap.dcMotor.get("rightfront");
        motorRightBack = hardwareMap.dcMotor.get("rightback");
        motorLeftFront = hardwareMap.dcMotor.get("leftfront");
        motorLeftBack = hardwareMap.dcMotor.get("leftback");
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLinearRight = hardwareMap.dcMotor.get("linearr");
        motorLinearLeft = hardwareMap.dcMotor.get("linearl");

        motorFeeder = hardwareMap.dcMotor.get("feeder");

        motorCatapult = hardwareMap.dcMotor.get("catapult");

        servo = hardwareMap.crservo.get("servo");
        servo2 = hardwareMap.crservo.get("servo2");
        servoFeeder = hardwareMap.servo.get("servofeeder");
        servoLinear = hardwareMap.servo.get("servolinear");
        servoLinear2 = hardwareMap.servo.get("servolinear2");

        ODSright = hardwareMap.opticalDistanceSensor.get("odsright");
        ODSleft = hardwareMap.opticalDistanceSensor.get("odsleft");


        //color = hardwareMap.colorSensor.get("color");

        servoLinear.setPosition(1);
        servoLinear2.setPosition(.5);
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
    @Override
    public void loop(){

        colorCcache = colorCreader.read(0x04, 1);

        beaconSet();
        /*
        Gamepad1's keymap                       Gamepad2's keymap
        DPAD                                    DPAD
            up                                      up
            down                                    down
            left                                    left
            right                                   right
        LEFT_STICK                              LEFT_STICK
            up                  -drive motor        up                  -ballfeed
            down                -drive motor        down                -ballfeed
            left                -drive motor        left
            right               -drive motor        right
        RIGHT_STICK                             RIGHT_STICK
            up                                     up                  -catapult
            down                                     down                -catapult
            left                -drive motor        left
            right               -drive motor        right
        BUTTONS                                 BUTTONS
            a                                       a
            x                                       x
            b                                       b
            y                                       y
        BUMPERS                                 BUMPERS
            left_trigger                            left_trigger        -linear motor
            left_bumper         -beacon             left_bumper
            right_trigger                           right_trigger       -linear motor
            right_bumper        -beacon             right_bumper

         */
        if(gamepad1.left_stick_button||gamepad1.right_stick_button){
            throttlecontrol = 0.35;
        }else throttlecontrol = 1;
        if (gamepad2.dpad_up){
            servoLinear.setPosition(.5);
            servoLinear2.setPosition(1);}
        if (gamepad2.dpad_left){
            servoLinear.setPosition(1);
            servoLinear2.setPosition(.5);}
        if (gamepad2.x)
            servoFeeder.setPosition(.3);
        if (gamepad2.y)
            servoFeeder.setPosition(0 );
        //Mecanum wheel drive - Vector Addition and subtraction
        float[] motorVals = {0,0,0,0};
        //Right Front
        motorVals[0] = -gamepad1.left_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_x;
        //Right Back
        motorVals[1] = -gamepad1.left_stick_y+gamepad1.right_stick_x-gamepad1.left_stick_x;
        //Left Front
        motorVals[2] = -gamepad1.left_stick_y-gamepad1.right_stick_x-gamepad1.left_stick_x;
        //Left Back
        motorVals[3] = -gamepad1.left_stick_y-gamepad1.right_stick_x+gamepad1.left_stick_x;
        //Adjust range to that allowed by DcMotors
        motorVals = map(motorVals,-1,1);
        //Set power to motors
        motorRightFront.setPower(motorVals[0]*throttlecontrol);
        motorRightBack.setPower(motorVals[1]*throttlecontrol);
        motorLeftFront.setPower(motorVals[2]*throttlecontrol);
        motorLeftBack.setPower(motorVals[3]*throttlecontrol);

        motorLinearLeft.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
        motorLinearRight.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

        motorFeeder.setPower(gamepad2.left_stick_y);
        motorCatapult.setPower(gamepad2.right_stick_y);
        //set servo positions/power
        power = 0;
        if(gamepad1.right_bumper){
            power = 1;
        }else if(gamepad1.left_bumper) power = -1;
        servo.setPower(power);
    }

    @Override
    public void stop(){

    }
    //you dont need to know what this does.
    public float[] map(float[] a, double minR, double maxR){
        float min=Float.MAX_VALUE;
        float max=Float.MIN_VALUE;
        for(int i = 0;i<=3;i++){
            if(a[i] < min){
                min = a[i];
            }
            if(a[i] > max){
                max = a[i];
            }
        }
        if(min>=-1&&max<=1){
            return a;
        }
        double scalemin = Math.abs(minR/min);
        double scalemax = Math.abs(minR/max);
        double finalscale;

        if(scalemin<scalemax)
            finalscale = scalemin;
        else
            finalscale = scalemax;
        for(int i = 0;i<=3;i++){
            a[i] = a[i]*(float)finalscale;
        }

        return a;
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

}