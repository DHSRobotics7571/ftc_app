package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name="Modules", group="Autonomous")
public class testingDifferentModules extends OpMode {
    //instantiate objects
    int robostate = 0;
    DcMotor motorRightFront,motorRightBack,motorLeftFront,motorLeftBack;
    DcMotor motorLinearRight,motorLinearLeft;
    DcMotor motorFeeder;
    DcMotor motorCatapult;

    CRServo servoBeacon;

    OpticalDistanceSensor ODSright, ODSleft;
    ColorSensor color;
    CRServo beaconpusher;

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

        servoBeacon = hardwareMap.crservo.get("beacon");

        ODSright = hardwareMap.opticalDistanceSensor.get("odsright");
        ODSleft = hardwareMap.opticalDistanceSensor.get("odsleft");

        beaconpusher = hardwareMap.crservo.get("beacon");

        color = hardwareMap.colorSensor.get("color");
        motorFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop() {
        switch (robostate) {
            case 0:
                long time = 0;
                setThrottle(.5);
                if (System.currentTimeMillis() - time >= 1000) {
                    setThrottle(0);
                    motorFeeder.setPower(1);
                    motorCatapult.setPower(1);
                    if (System.currentTimeMillis() - time >= 5000) {
                        motorCatapult.setPower(0);
                    }
                }
                robostate++;
                break;
            default:
                break;

        }



    }


    @Override
    public void stop () {

        setThrottle(0);
    }


    //square robot with the wall to push the beacons

    /*
        If power is positive, goes forward
        If power is negative, goes backwards
     */
    public void setThrottle(double power){
        motorRightBack.setPower(power);
        motorRightFront.setPower(-power);
        motorLeftBack.setPower(power);
        motorLeftFront.setPower(-power);
    }
    /*
        If power is positive, turns right
        If power is negative, turns left
     */
    public void turn(double power){
        motorRightBack.setPower(-power);
        motorRightFront.setPower(-power);
        motorLeftBack.setPower(power);
        motorLeftFront.setPower(power);
    }
    /*
        If power is positive, strafes right
        If power is negative, strafes left
     */
    public void strafe(double power){
        motorRightBack.setPower(power);
        motorRightFront.setPower(-power);
        motorLeftBack.setPower(power);
        motorLeftFront.setPower(-power);
    }

}
