package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Autonomous ffs", group="Autonomous")
@Disabled
public class AUTOMOUS_FINAL extends OpMode {
    int robostate = 1;
    //instantiate objects
    DcMotor motorRightFront,motorRightBack,motorLeftFront,motorLeftBack;
    DcMotor motorLinearRight,motorLinearLeft;
    DcMotor motorFeeder;
    DcMotor motorCatapult;

    CRServo servoBeacon;
    Servo servofeeder;
    Servo servoLinear;
    Servo servoLinear2;
    double throttlecontrol = 1;
    OpticalDistanceSensor ODSright, ODSleft;
    ColorSensor color;


    //logic objects
    double power;

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
        servofeeder = hardwareMap.servo.get("servofeeder");
        servoLinear = hardwareMap.servo.get("servolinear");
        servoLinear2 = hardwareMap.servo.get("servolinear2");

        ODSright = hardwareMap.opticalDistanceSensor.get("odsright");
        ODSleft = hardwareMap.opticalDistanceSensor.get("odsleft");


        color = hardwareMap.colorSensor.get("color");

        servoLinear.setPosition(1);
        servoLinear2.setPosition(.5);
    }
    long time;
    @Override
    public void loop() {
        telemetry.addData("Ticks",motorLeftBack.getCurrentPosition());
        //3656 ticks to firing plane
        switch (robostate) {
            case 1:
                motorCatapult.setPower(-1);
                time = System.currentTimeMillis();
                robostate++;
                break;
            case 2:
                if(System.currentTimeMillis()-time>=3000){
                    motorCatapult.setPower(0);
                    robostate++;

                }
                break;

        }

    }
    public void setThrottle(double throttle){
        motorLeftBack.setPower(throttle);
        motorLeftFront.setPower(throttle);
        motorRightBack.setPower(throttle);
        motorRightFront.setPower(throttle);
    }
    @Override
    public void stop(){
        motorCatapult.setPower(0);
    }
}
