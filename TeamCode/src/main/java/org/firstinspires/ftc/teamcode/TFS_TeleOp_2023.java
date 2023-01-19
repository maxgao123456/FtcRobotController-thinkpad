/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "TFS_TeleOp", group = "FTC2023")
//@Disabled
public class TFS_TeleOp_2023 extends LinearOpMode
{
    boolean                 inDevelopment = true;
    boolean                 isPratice = false;
    double                  wrist_Tilt_Offset = -0.025;
    double                  wrist_Flip_Offset = 0.01;
    double                  claw_Left_Offset =  0;
    double                  claw_Right_Offset = 0;

    int                     isLeftSide = 0;
    double                  coneDropPanAngle = -28;
    double                  coneDropLiftAngle = -35;
    double                  coneDropExtendLength = 0.275;
    double                  conePickupPanAngle = 0;
    double                  conePickupLiftAngle = 97;
    double                  conePickupExtendLength = 0.32;
    boolean                 coneLiftingMissionCompleted = true;
    boolean                 conePickupMissionCompleted = false;



    // Hardware Devices
    BNO055IMU               imu;
    DcMotor                 LFMotor;
    DcMotor                 RFMotor;
    DcMotor                 LRMotor;
    DcMotor                 RRMotor;
    DcMotorEx               Arm_PanMotor, Arm_lift_Left_Motor, Arm_lift_Right_Motor;
    DcMotor                 Arm_ExtendMotor;
    Servo                   ClawServo_Left,ClawServo_Right, Wrist_FlipServo, Wrist_TiltServo;
    TouchSensor             arm_extendSwitch;
    DistanceSensor          rightd,leftd,centerd;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;
    int                     lfPosition_last =0, rfPosition_last =0,lrPosition_last =0,rrPosition_last =0;
    double[]                robotPosition = new double[]{0,0,0};
    private ElapsedTime     runtime = new ElapsedTime();
    int                     objectDetectionCount =0;

    /******************************Robot & Field Specific Constants ********************************/
    double                  wheel_diameter = 0.095, encoderPPR = 384.5; // 1150RPM with 1:2 Gear downdouble                  turret_Pulse_per_Degree = 384.5*2*70/10/360.0; //version 435RPM Motor with 1:2 bevel gear and 10T to 70T gear chain
    double                  turret_Pulse_per_Degree = 384.5*2*70/10/360.0; //version 435RPM Motor with 1:2 bevel gear and 10T to 70T gear chain
    double                  lift_Pulse_per_Degree = 751.8*2*42/10/360.0; //version 435RPM Motor with 1:2 bevel gear and 10T to 70T gear chain
    double                  Arm_extend_distanceRatio = 2952/0.88; //max extension 74.5cm
    int                     servo_rest_position = 110; //wrist servo max position
    double                  maxExtendArm = 0.6;
    double                  minExtendArm = 0.01;
    double                  speedExtend = 0.1;
    double                  speedLift = 15;
    double                  speedPan = 15;

    double                  maxWristTiltArm = 135;
    double                  minWristTiltArm = -135;
    double                  maxWristFlipArm = 135;
    double                  minWristFlipArm = -135;
    double                  speedWristTilt = 10;
    double                  maxPanArm = 270;
    double                  minPanArm = -270;
    double                  maxLiftArm = 140;
    double                  minLiftArm = -140;
    double                  maxClawArm = 90;
    double                  minClawArm = -30;

    double                  armLiftResolution = 1; //Arm_lift position control resolution
    double                  armPanResolution = 1; //Arm_Pan position control resolution
    double                  armExtendResolution = 0.0025; //Arm_Extend position control resolution

    double                  control1SpeedFactor = 0.5;
    double                  control2SpeedFactor = 0.5;

    /*********************************** Thread Tasks   *************************************/

    goXYnTurnThread goXYnTurnThread = null;
    Arm_Lift_PositionThread  arm_liftThread = null;
    Arm_Pan_PositionThread  arm_panThread = null;
    Arm_Extend_PositionThread arm_extendThread = null;
    ScanConeThread scanCone = null;
    ScanJunctionThread scanJunction = null;


    /************************************** Task and thread control Flags ******************************/
    boolean                 chassisAllowInAuto = true;
    boolean                 isInAuto = false; // during tele-op, set this to true in your task thread, this prevent multiple semi-auto task
    boolean                 isAutoEnable = true;        // To cancel the auto task, set this to false
    boolean                 Chassis_motorEnabled = true;    // to enable and cancel chassis task
    boolean                 Arm_pan_motorEnabled = true;    // to enable and cancel Turret Pan task
    boolean                 Arm_lift_motorEnabled = true;   // to enable and cancel Arm Lift task
    boolean                 Arm_extend_motorEnabled = true; // to enable and cancel Arm Extend task
    boolean                 scanFlag = false;

    /******************************set the tel-op holding position *************************************/
    double end_arm_tilt_position;
    double end_arm_pan_position;
    double end_arm_extend_position;
    double end_wrist_tilt_position;

    int stackCount = 0;

    @Override public void runOpMode() {
        telemetry.addData(":@@@@@@@@@@@@@@@@@@@@@@@ ","" );
        telemetry.addLine();
        telemetry.addData("Press X for ","Left Side" );
        telemetry.addData("Press B for ","Right Side" );
        telemetry.addLine();
        telemetry.addData(":@@@@@@@@@@@@@@@@@@@@@@@@ ","" );

        telemetry.update();

        while (isLeftSide ==0 && !isStopRequested())
        {
            if (gamepad1.x || gamepad2.x) {
                isLeftSide = 1;
                telemetry.addData("Program set to ","Left Side" );
                telemetry.update();
                break;
            }
            if (gamepad1.b || gamepad2.b) {
                isLeftSide = -1;
                telemetry.addData("Program set to ","Right Side" );
                telemetry.update();
                break;
            }
            sleep(50);
        }
        if (isLeftSide == -1) // right side
        {   coneDropPanAngle = 29;
            coneDropLiftAngle = -35;
            coneDropExtendLength = 0.275;}

        Init();

        // ??????????? not needed?
        resetAngle();
        //resetEncoders();
        //
        telemetry.addData("WAITING FOR ", "START");
        telemetry.update();

        waitForStart();
        armUpRightPosition();

        //????????????????????? not needed??
        // Hold the robot at the current position or go to a desired position , otherwise tele-operation will move the arm
//        Arm_Lift_Position(getArmLiftPosition(), 0.5);
//        Arm_Pan_position(getArmPanPosition(),0.5);
//        Arm_Extend_Position(getArmExtendPosition(),0.5);

        boolean buttonClick = false;
        //boolean oneController = false;

        while (opModeIsActive()) {

            //chassis drive
            // POV Mode uses left joystick to go forward (X) & strafe (Y), and right joystick to rotate.
            // Note: pushing stick forward gives negative value

            double axial =  -(control1SpeedFactor * gamepad1.left_stick_y);
            double lateral = (control1SpeedFactor * gamepad1.left_stick_x);
            double yaw = 0.5* (control1SpeedFactor * gamepad1.right_stick_x);

            double arm_pan_cruise_speed     =       -gamepad2.left_stick_x*control2SpeedFactor;
            double arm_lift_cruise_speed    =       gamepad2.left_stick_y*control2SpeedFactor;
            double arm_extend_cruise_speed  =       -gamepad2.right_stick_y*control2SpeedFactor;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

/*************************  Semi-auto Tasks come here ****************************/


            if (gamepad2.y ||gamepad1.y) {
                if (!isInAuto){
                    isInAuto = true;
                    teleConeToHighPoleTask();
                    ;                    }//Threaded Task
            }
            if (gamepad2.x|| gamepad1.x) {
                if (!isInAuto){
                    isInAuto = true;
                    armUpRightPositionTask();
                }//Threaded Task
            }
            if (gamepad1.a) {
                if (!isInAuto){
                    isInAuto = true;
                    scanextend(26);
                    isInAuto = false;

                    //telePickupFromSubStation();
                }//Threaded Task
            }
            if(gamepad2.a){
                if (!isInAuto){
                    isInAuto = true;
                    //claw_Scan();
                    scanJunction = new ScanJunctionThread(-35,-16);
                    scanJunction.start();
                    while (scanJunction.isAlive() ) idle();
                    //scanextend(26);
                    telemetry.addData("Scanthread:",scanJunction.isAlive() );
                    telemetry.addData("finished","");
                    telemetry.update();
                    isInAuto = false;
                }

            }

            if (gamepad2.b || gamepad1.b) { // drop the cone into the pole
                wrist_cone_front_drop(0);
            }

            // Cancel Semi-auto task with "back"" button
            if (gamepad2.back || gamepad1.back)
            {
                isAutoEnable = false;
            }

            if (gamepad1.left_bumper && gamepad1.right_bumper ) // automatically cycle the pickup and drop
            {
                if (!isInAuto){

                    if (coneLiftingMissionCompleted) {
                        wrist_cone_front_drop(0); //drop the cone
                        sleep(200);
                        isInAuto = true;
                        telePickupFromSubStation();
                    }
                    if (conePickupMissionCompleted) {
                        isInAuto = true;
                        teleConeToHighPoleTask();
                    }
                }//Threaded Task
            }

            if (gamepad1.start && gamepad1.right_bumper ) // store the robot position for cone dropping and lock the chassis
            {
                if (Math.abs(getArmLiftPosition()) < 70)  // to prevent operation mistake
                {
                    coneDropPanAngle = getArmPanPosition();
                    coneDropLiftAngle = getArmLiftPosition();
                    coneDropExtendLength = getArmExtendPosition();
                    brakeAll(true,false);// lock the robot in place
                }
            }

            if (gamepad1.start && gamepad1.left_bumper ) // store the robot position for cone dropping and lock the chassis
            {
                if (Math.abs(getArmLiftPosition()) >70)  // to prevent operation mistake
                {
                    telemetry.addData("storing new dropping value", "!");
                    telemetry.update();
                    conePickupPanAngle = getArmPanPosition();
                    conePickupLiftAngle = getArmLiftPosition();
                    conePickupExtendLength = getArmExtendPosition();
                    brakeAll(true, false);// lock the robot in place
                }
            }



/**********************  Tele-Operation **********************************************************/
            if (!isInAuto || chassisAllowInAuto) //allowing chassis to move during semi-auto while excluding the arm from tel-operation
            {
                //elemetry.addData("In", "Tele-op");
                //telemetry.update();
                // Chassis Manual Control
                LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                MotorsPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }

            if (!isInAuto)  //arm is only allow to move when not in semi-auto task to avoid conflict
            {
                //telemetry.addData("In", "Tele-op");
                //telemetry.update();
                end_arm_pan_position = getArmPanPosition();
                if ((arm_pan_cruise_speed > 0.1  && end_arm_pan_position < maxPanArm) || (arm_pan_cruise_speed < -0.1  && end_arm_pan_position > minPanArm ))
                {
                    double targetPanArm = end_arm_pan_position + arm_pan_cruise_speed * speedPan *control2SpeedFactor;
                    if (targetPanArm > maxPanArm) targetPanArm = maxPanArm;
                    if (targetPanArm < minPanArm) targetPanArm = minPanArm;
                    Arm_PanMotor.setTargetPosition((int) (targetPanArm *turret_Pulse_per_Degree));
                    Arm_PanMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm_PanMotor.setPower(arm_pan_cruise_speed*control2SpeedFactor);
                } else Arm_PanMotor.setPower(0.25); // to hold the motor in position

                end_arm_tilt_position = getArmLiftPosition();
                if (arm_lift_cruise_speed > 0.1 && end_arm_tilt_position <  maxLiftArm  || arm_lift_cruise_speed < -0.1 && end_arm_tilt_position > minLiftArm  )
                {
                    double targetLiftArm = end_arm_tilt_position + arm_lift_cruise_speed * speedLift*control2SpeedFactor;
                    if (targetLiftArm > maxLiftArm) targetLiftArm = maxLiftArm;
                    if (targetLiftArm < minLiftArm) targetLiftArm = minLiftArm;

                    Arm_lift_Left_Motor.setTargetPosition((int) (targetLiftArm *lift_Pulse_per_Degree));
                    Arm_lift_Right_Motor.setTargetPosition((int) (targetLiftArm *lift_Pulse_per_Degree));

                    Arm_lift_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm_lift_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm_lift_Left_Motor.setPower(arm_lift_cruise_speed*control2SpeedFactor );
                    Arm_lift_Right_Motor.setPower(arm_lift_cruise_speed*control2SpeedFactor );
                } else
                {
                    Arm_lift_Left_Motor.setPower(0.5);  // to hold the motor in position
                    Arm_lift_Right_Motor.setPower(0.5);
                }

                end_arm_extend_position = getArmExtendPosition();
                if ((arm_extend_cruise_speed > 0.1 && end_arm_extend_position < maxExtendArm ) || arm_extend_cruise_speed < -0.1 && end_arm_extend_position > minExtendArm  )
                {
                    double targetExtendArm = end_arm_extend_position + arm_extend_cruise_speed * speedExtend * control2SpeedFactor;
                    if (targetExtendArm > maxExtendArm) targetExtendArm = maxExtendArm;
                    if (targetExtendArm < minExtendArm) targetExtendArm = minExtendArm;
                    Arm_ExtendMotor.setTargetPosition((int) (targetExtendArm *Arm_extend_distanceRatio));
                    Arm_ExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm_ExtendMotor.setPower(arm_extend_cruise_speed*control2SpeedFactor);

                } else
                {
                    Arm_ExtendMotor.setPower(0.25); // to hold the motor in position
                }

                // Wrist Servo Tilt Up and Down
                end_wrist_tilt_position = getArmWristTiltPosition();
                if (((gamepad2.left_trigger - gamepad2.right_trigger) > 0.1 && end_wrist_tilt_position < maxWristTiltArm ) || ((gamepad2.left_trigger - gamepad2.right_trigger) < -0.1 && end_wrist_tilt_position > minWristTiltArm))
                {
                    double targetWristTiltArm = end_wrist_tilt_position + (gamepad2.left_trigger - gamepad2.right_trigger) * speedWristTilt * control2SpeedFactor;
                    if (targetWristTiltArm > maxWristTiltArm) targetWristTiltArm = maxWristTiltArm;
                    if (targetWristTiltArm < minWristTiltArm) targetWristTiltArm = minWristTiltArm;

                    wrist_Tilt_Servo( targetWristTiltArm);


                }

                // Wrist Servo Flip back and Forward
                if(gamepad2.left_bumper){
                    wrist_Flip_Servo(90);
                }
                if(gamepad2.right_bumper){
                    wrist_Flip_Servo(-90);
                }

                // Claw Open and CLose
                if(gamepad2.start){
                    claw_Servo(90);//open
                }
                if(gamepad2.back){
                    claw_Servo(30);//close
                }

//                if(gamepad1.start){
//                    if(!buttonClick){
//                        buttonClick = true;
//                        oneController = !oneController;
//                    }
//                }
//                else{
//                    buttonClick = false;
//                }

                // Change the tele-operation speed for the Chassis and Arm
                if (gamepad2.dpad_up) {
                    control2SpeedFactor = 1;
                } else if (gamepad2.dpad_right) {
                    control2SpeedFactor = 0.75;
                } else if (gamepad2.dpad_down) {
                    control2SpeedFactor = 0.5;
                } else if (gamepad2.dpad_left) {
                    control2SpeedFactor = 0.3;
                }

                if (gamepad1.dpad_up) {
                    control1SpeedFactor = 1;
                } else if (gamepad1.dpad_right) {
                    control1SpeedFactor = 0.75;
                } else if (gamepad1.dpad_down) {
                    control1SpeedFactor = 0.5;
                } else if (gamepad1.dpad_left) {
                    control1SpeedFactor = 0.35;
                }

                if(gamepad1.left_trigger > 0.2){
                    wrist_cone_front_ready(0);
                }
                if(gamepad1.right_trigger > 0.2){
                    wrist_cone_front_drop(0);
                }


                telemetry.addData("Controller1 Speed", control1SpeedFactor);
                telemetry.addData("Controller2 Speed", control2SpeedFactor);
                telemetry.addData("FrontLeft Wheel Pos", LFMotor.getCurrentPosition());
                telemetry.addData("pan pos",getArmPanPosition());
                telemetry.addData("right:", String.format("%.01f mm", rightd.getDistance(DistanceUnit.MM)));
                telemetry.addData("left:", String.format("%.01f mm", leftd.getDistance(DistanceUnit.MM)));
                telemetry.addData("center:", String.format("%.01f mm", centerd .getDistance(DistanceUnit.MM)));
                telemetry.update();
            }
            sleep(50);
        }
    }

    /*************** Utilities for Resetting *******************************/
    void Init()        {

        //Robot (1) Configuration:
        // Control Hub: Motors: {lf, rf, lr, rr}  || I2C(0): {imu,base_color} || Digital: {turret} // Servos:
        // Expansion Hub: Motors: {arm_pan, arm_lift, arm_extend, duck_spin}  || I2C(0): {imu_arm,intake} || digital: arm_extend || Servos: {intakeServo1,intakeServo2, armServo1, armServo2, cameraPanServo, cameraTiltServo }

        LFMotor = hardwareMap.dcMotor.get("lf");
        RFMotor = hardwareMap.dcMotor.get("rf");
        LRMotor = hardwareMap.dcMotor.get("lr");
        RRMotor = hardwareMap.dcMotor.get("rr");
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        LRMotor.setDirection(DcMotor.Direction.REVERSE);
        RRMotor.setDirection(DcMotor.Direction.FORWARD);
        RRMotor.setDirection(DcMotor.Direction.FORWARD);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm_PanMotor = hardwareMap.get(DcMotorEx.class, "Turret");
        Arm_PanMotor.setDirection(DcMotor.Direction.FORWARD); //version 1, reverse
        Arm_PanMotor.setPositionPIDFCoefficients(3);

        Arm_lift_Left_Motor = hardwareMap.get(DcMotorEx.class, "Left_Shoulder");
        Arm_lift_Left_Motor.setPositionPIDFCoefficients(3);
        Arm_lift_Right_Motor = hardwareMap.get(DcMotorEx.class, "Right_Shoulder");
        Arm_lift_Right_Motor.setPositionPIDFCoefficients(3);
        Arm_lift_Left_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm_lift_Right_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        Arm_ExtendMotor  = hardwareMap.dcMotor.get("Extension");
        Arm_ExtendMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Arm_PanMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_lift_Left_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_lift_Right_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_ExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ClawServo_Left = hardwareMap.get(Servo.class, "Left_Claw");
        ClawServo_Right = hardwareMap.get(Servo.class, "Right_Claw");
        Wrist_TiltServo = hardwareMap.get(Servo.class, "Lift");
        Wrist_FlipServo = hardwareMap.get(Servo.class, "Flip");
        Wrist_TiltServo.setDirection(Servo.Direction.REVERSE);
        Wrist_FlipServo.setDirection(Servo.Direction.REVERSE);

        ClawServo_Left.setDirection(Servo.Direction.FORWARD);
        ClawServo_Left.setDirection(Servo.Direction.REVERSE);

        arm_extendSwitch = hardwareMap.get(TouchSensor.class,"Touch");
        rightd = hardwareMap.get(DistanceSensor.class, "Left_D");
        centerd = hardwareMap.get(DistanceSensor.class, "Center_D");
        leftd = hardwareMap.get(DistanceSensor.class, "Right_D");
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Check if the joystick is operating properly
        while (Math.abs(gamepad1.left_stick_x) >0.01 || Math.abs(gamepad1.left_stick_y) >0.01 || Math.abs(gamepad2.left_stick_x) >0.01 || Math.abs(gamepad2.left_stick_y) >0.01 || Math.abs(gamepad1.right_stick_x) >0.01 || Math.abs(gamepad1.right_stick_y) >0.01 ||Math.abs(gamepad2.right_stick_x) >0.01 ||Math.abs(gamepad2.right_stick_y) >0.01 )
        {
            telemetry.addData(" Gamepad ERROR" , "Retart the Driver Hub");
            telemetry.addData(" Gamepad ERROR" , "Retart the Driver Hub");
            telemetry.addData(" Gamepad ERROR" , "Retart the Driver Hub");
            telemetry.addData(" Gamepad ERROR" , "Retart the Driver Hub");
            telemetry.addData(" Gamepad ERROR" , "Retart the Driver Hub");
            telemetry.update();
        }
        //************************************************
        telemetry.addData(" Gamepad Ready" , "!");
        telemetry.update();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating Imu...");

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("Initialization Completed", "!");

        sleep(500);


    }
    public void testscan(){
        double ctrlValue = 3;
        boolean foundCone;
        double scanAngle = getArmPanPosition();
        while(opModeIsActive() )
        {
            double leftDis = leftd.getDistance(DistanceUnit.MM);
            double rightDis = rightd.getDistance(DistanceUnit.MM);

            if ((leftDis < 140 || rightDis < 140)&& Math.abs((leftDis - rightDis)) < 10){
                foundCone = true;
            }
            else{
                scanAngle += ctrlValue;
                Arm_PanMotor.setTargetPosition((int) (scanAngle * turret_Pulse_per_Degree));
                Arm_PanMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm_PanMotor.setPower(scanAngle);
                foundCone = false;
            }
            telemetry.addData("hLeftDis", "%.2f", leftd.getDistance(DistanceUnit.MM));
            telemetry.addData("hRightDis", "%.2f", rightd.getDistance(DistanceUnit.MM));
            telemetry.addData("cone detected",foundCone);
            telemetry.update();
        }
    }
    private void resetTurret ()        {
        Arm_PanMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turret reset done
    }

    void resetLiftArm()        {
        Arm_lift_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_lift_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void resetArm_Extend ()        {
        Arm_ExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (!arm_extendSwitch.isPressed())
        {Arm_ExtendMotor.setPower(-0.5);}
        Arm_ExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_ExtendMotor.setTargetPosition(10);
        Arm_ExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_ExtendMotor.setPower(0.15);
    }

    private void localization (){
        //estimate robot location on the field using forward kinematic, DOES NOT assume "no turning"
        double Y_Adjustment = 0.85; // lost of distance due to slipping
        int deltalfPosition = LFMotor.getCurrentPosition()-lfPosition_last;
        int deltarfPosition = RFMotor.getCurrentPosition()-rfPosition_last;
        int deltalrPosition = LRMotor.getCurrentPosition()-lrPosition_last;
        int deltarrPosition = RRMotor.getCurrentPosition()-rrPosition_last;

        lfPosition_last += deltalfPosition;
        rfPosition_last += deltarfPosition;
        lrPosition_last += deltalrPosition;
        rrPosition_last += deltarrPosition;

        double angle = Math.toRadians(getAngle());

        double deltaXLocal = 0.25 * (deltalfPosition + deltarfPosition + deltalrPosition + deltarrPosition) / encoderPPR * 3.14 * wheel_diameter;
        double deltaYLocal = Y_Adjustment * 0.25 * (-deltalfPosition + deltarfPosition + deltalrPosition - deltarrPosition) / encoderPPR * 3.14 * wheel_diameter;

        double deltaXGlobal = Math.cos(angle)*deltaXLocal - Math.sin(angle) * deltaYLocal;
        double deltaYGlobal = Math.sin(angle)*deltaXLocal + Math.cos(angle) * deltaYLocal;

        robotPosition[0]+= deltaXGlobal;
        robotPosition[1]+= deltaYGlobal;
        robotPosition[2] = Math.toDegrees(angle);
//            telemetry.addData("X:    ", robotPosition[0]);
//            telemetry.addData("Y:    ", robotPosition[1]);
//            telemetry.addData("Angle:", robotPosition[2]);
//            telemetry.update();
    }

    /*************************** IMU & Angle Utilities*************************/
    private void resetAngle ()        {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle ()        {
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }


    /**************************Navigation and Chassis Control ************************************/

    private void resetEncoders (){
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Resetting Localization memory
        lfPosition_last =0; rfPosition_last=0; lrPosition_last=0; rrPosition_last=0;

        //resetting global position
        robotPosition [0]=0;
        robotPosition [1]=0;

    }

    private void brakeAll (boolean isLockPosition, boolean isBrakeOn){

        if (isLockPosition)
        {
            LFMotor.setTargetPosition(LFMotor.getCurrentPosition());
            RFMotor.setTargetPosition(RFMotor.getCurrentPosition());
            LRMotor.setTargetPosition(LRMotor.getCurrentPosition());
            RRMotor.setTargetPosition(RRMotor.getCurrentPosition());
            LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LFMotor.setPower(0.3);
            RFMotor.setPower(0.3);
            LRMotor.setPower(0.3);
            RRMotor.setPower(0.3);
        } else if (isBrakeOn)
        {
            LFMotor.setPower(0);
            LRMotor.setPower(0);
            RFMotor.setPower(0);
            RRMotor.setPower(0);
        }
    }
    private void MotorsPower ( double LFPower, double RFPower, double LRPower, double RRPower)
    {
        double max;
        max = Math.max(Math.abs(LFPower), Math.abs(RFPower));
        max = Math.max(max, Math.abs(LRPower));
        max = Math.max(max, Math.abs(RRPower));

        if (max > 1.0) { //Normalized
            LFPower /= max;
            RFPower /= max;
            LRPower /= max;
            RRPower /= max;
        }
        LFMotor.setPower(LFPower*control1SpeedFactor);
        RFMotor.setPower(RFPower*control1SpeedFactor);
        LRMotor.setPower(LRPower*control1SpeedFactor);
        RRMotor.setPower(RRPower*control1SpeedFactor);
    }
    private double speed_planning ( double V_max, double current_d, double target_d)
    {
        double V1, V3;
        double V_start = 0.2; //starting power without getting wheel slip
        double V_end = 0.1; //end speed to approach destination

        double Kup = 8, Kdown = 2/ (Math.abs(target_d) + 2.5); // Acc and de-acc rate  Power/count
        if (target_d > 0) {
            V1 = V_start + Kup * current_d;
            if (V1 < V_start) V1= V_start;
            V3 = V_end + Kdown * (target_d - current_d);
            if (V3<V_end) V3 = V_end;

            return Math.min(V1, Math.min(V_max, V3));
        } else {
            V1 = -V_start + Kup * current_d;
            if (V1 > -V_start) V1= -V_start;
            V3 = -V_end + Kdown * (target_d - current_d);
            if (V3>-V_end) V3 = -V_end;
            return Math.max(V1, Math.max(-V_max, V3));
        }
    }

    private boolean goXYnTurn ( double heading, double targetX, double targetY, double V_max,
                                double Tracking_resolution, double V_max_Turn, double turn_resolution, int wallEnable, boolean isLockPosition, boolean isBrakeOn){
        Chassis_motorEnabled = true;
        double drive_powerX = 0, drive_powerY=0;
        //resetEncoders();
        double x0 = robotPosition[0], y0= robotPosition[1];
        double V_XLocal=0, V_YLocal=0, Turn_power=0;
        double distance2Target = Math.hypot((targetX - robotPosition[0]),(targetY-robotPosition[1]));
        double distance2Go;
        boolean isTargetPosArrived = false, isHeadingAchieved = false;
        double traveledDistance =0;
        double Vp;
        double pathAngle =0;
        double robotHeading =0;
        double Kp = 0.003; // Proportional gain for turning control 0.01 means full power(speed) at 100 degree turn
        double V_turn_min = 0.04;
        double angle2turn;

        while (Chassis_motorEnabled  && !isStopRequested() && (!isHeadingAchieved || !isTargetPosArrived)) {
            distance2Go = Math.hypot((targetX - robotPosition[0]),(targetY-robotPosition[1]));
            angle2turn = heading - getAngle();

            if (distance2Go  > Tracking_resolution && traveledDistance < distance2Target )
            {//distance to target spot
                traveledDistance = Math.hypot((robotPosition[0] - x0), (robotPosition[1] - y0));
                Vp = speed_planning(V_max, traveledDistance, distance2Target  - Tracking_resolution );
                pathAngle = Math.atan2(targetY - robotPosition[1], targetX - robotPosition[0]);
                robotHeading = Math.toRadians(getAngle());
                V_XLocal = Vp * Math.cos(robotHeading - pathAngle);
                V_YLocal = -Vp * Math.sin(robotHeading - pathAngle);
                isTargetPosArrived = false;
            } else
            {V_XLocal =0;V_YLocal =0; isTargetPosArrived = true;}

            if (Math.abs(angle2turn) > turn_resolution)
            {//Turning to target heading
                Turn_power = Kp * angle2turn;

                if (Math.abs(Turn_power) < V_turn_min) {
                    Turn_power = V_turn_min * Math.signum(Turn_power);
                }
                if (Math.abs(Turn_power) > V_max_Turn) {
                    Turn_power = V_max_Turn * Math.signum(Turn_power);
                }
                isHeadingAchieved = false;
            } else {Turn_power =0; isHeadingAchieved = true;}

            LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double LFPower = V_XLocal - V_YLocal - Turn_power ;
            double RFPower = V_XLocal + V_YLocal + Turn_power ;
            double LRPower = V_XLocal + V_YLocal - Turn_power ;
            double RRPower = V_XLocal - V_YLocal + Turn_power ;

            MotorsPower(LFPower, RFPower, LRPower, RRPower);
        }
        brakeAll(isLockPosition, isBrakeOn);
        return true;
    }

    /*********************Arm Control ****************/
    private void Arm_Pan_position ( double target, double power){
        Arm_pan_motorEnabled = true;
        if (target <minPanArm) target =  minPanArm;
        if (target > maxPanArm) target = maxPanArm;

        Arm_PanMotor.setTargetPosition((int) (target * turret_Pulse_per_Degree));
        Arm_PanMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_PanMotor.setPower(power);

        while((Arm_pan_motorEnabled && Math.abs(target-getArmPanPosition()) > armPanResolution) && !isStopRequested()) // wait until finish
        {
            //we could do something here to prevent getting stuck
            idle();
        }
        Arm_PanMotor.setPower(0.2);
    }
    private void Arm_Lift_Position ( double target, double power){
        Arm_lift_motorEnabled = true;
        if (target <minLiftArm) target = minLiftArm;
        if (target > maxLiftArm) target = maxLiftArm;

        Arm_lift_Left_Motor.setTargetPosition((int) (target * lift_Pulse_per_Degree));
        Arm_lift_Right_Motor.setTargetPosition((int) (target * lift_Pulse_per_Degree));
        Arm_lift_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_lift_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Arm_lift_Left_Motor.setPower(power);
        Arm_lift_Right_Motor.setPower(power);

        while ((Arm_lift_motorEnabled && Math.abs(target-getArmLiftPosition()) > armLiftResolution) && !isStopRequested()) {
            idle();
            //we could do something here to prevent getting stuck
        }
        // Hold the arm at the current position after target is reached
        Arm_lift_Left_Motor.setPower(0.5); // the 0 brake power is not enough
        Arm_lift_Right_Motor.setPower(0.5); // the 0 brake power is not enough
    }

    private void Arm_Extend_Position (double target, double power)        { //target in meter
        Arm_extend_motorEnabled = true;
        if (target <minExtendArm) target = minExtendArm;
        if (target > maxExtendArm) target = maxExtendArm;

        Arm_ExtendMotor.setTargetPosition((int) (target *Arm_extend_distanceRatio));
        Arm_ExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_ExtendMotor.setPower(power);

        while ((Arm_extend_motorEnabled && Math.abs(target-getArmExtendPosition()) > armExtendResolution) && !isStopRequested())
        {
            idle();
        }
        Arm_ExtendMotor.setPower(0.2); // the 0 brake power is not enough
    }

    private void wrist_Tilt_Servo ( double target)        { //gobilda servo 0.33 per 90 degrees
        if (target > maxWristTiltArm) target = maxWristTiltArm;
        if (target < minWristTiltArm) target = minWristTiltArm;
        double servoIncrement = target / 90 * 0.33;
        Wrist_TiltServo.setPosition(0.5 + wrist_Tilt_Offset + servoIncrement);
    }

    private void wrist_Flip_Servo ( double target)        { //gobilda servo 0.33 per 90 degrees
        if (target > maxWristFlipArm) target = maxWristFlipArm;
        if (target < minWristFlipArm) target = minWristFlipArm;
        double servoIncrement = target / 90 * 0.33;
        Wrist_FlipServo.setPosition(0.5 + wrist_Flip_Offset + servoIncrement);
    }

    private void claw_Servo (double target)
    {
        if (target > maxClawArm) target = maxClawArm;
        if (target < minClawArm) target = minClawArm;
        double servoIncrement = target / 90 * 0.33;
        ClawServo_Left.setPosition(0.5 + claw_Left_Offset + servoIncrement);
        ClawServo_Right.setPosition(0.5 + claw_Right_Offset + servoIncrement);
    }
    private void claw_Close() {claw_Servo(30    );}
    private void claw_Scan(){ ClawServo_Left.setPosition(0.9);ClawServo_Right.setPosition(0.9);}
    private void claw_Open() {claw_Servo(90);}

    // to drop the cone at front
    private void wrist_cone_front_ready(double adjustmentAngle)
    { // maintain 45 degree above level
        wrist_Flip_Servo(90);
        wrist_Tilt_Servo (-80-getArmLiftPosition() + adjustmentAngle);
    }
    private void wrist_cone_front_drop(double adjustmentAngle)
    { // maintain 15 degree below level
        wrist_Tilt_Servo (-150-getArmLiftPosition() + adjustmentAngle);
        sleep(250);
        claw_Open();
        sleep(100);
        //lift up the wrist by 90
        wrist_Tilt_Servo (-60-getArmLiftPosition() + adjustmentAngle);

    }

    // to pick up the cone fron the back during tele-op
    private void wrist_cone_pickup_back_ready(double adjustmentAngle)
    { // maintain  level
        wrist_Flip_Servo(-90);
        wrist_Tilt_Servo (45-getArmLiftPosition() + adjustmentAngle);
    }
    private void wrist_cone_pickup_back_Up(double adjustmentAngle)
    { // above level by 60
        wrist_Flip_Servo(-90);
        wrist_Tilt_Servo (-15-getArmLiftPosition() + adjustmentAngle);
    }

    private double getArmPanPosition(){return Arm_PanMotor.getCurrentPosition() / turret_Pulse_per_Degree;}
    private double getArmLiftPosition(){return (Arm_lift_Left_Motor.getCurrentPosition() + Arm_lift_Right_Motor.getCurrentPosition() ) /2.0/ lift_Pulse_per_Degree;}
    private double getArmExtendPosition(){return Arm_ExtendMotor.getCurrentPosition() / Arm_extend_distanceRatio;}
    private double getArmWristTiltPosition(){ return 90*(Wrist_TiltServo.getPosition()-(0.5+ wrist_Tilt_Offset))/0.33; }
    private double getArmWristFlipPosition(){ return 90*(Wrist_FlipServo.getPosition()-(0.5+ wrist_Flip_Offset))/0.33; }
    public void scanToFindCone()
    {
        scanCone = new ScanConeThread();
        scanFlag = true;
        scanCone.start();
    }
    public void scanextend(double targetdis){
        if(scanJunction.isFoundJunction()&&targetdis<60){
            double middleDis = centerd.getDistance(DistanceUnit.MM) - 22;
            double diffrence = (middleDis-targetdis)/1000 ;
            telemetry.addData("dis",diffrence);
            telemetry.update();
            arm_extendThread = new Arm_Extend_PositionThread(getArmExtendPosition()+diffrence, 0.9, 0);
            arm_extendThread.start();
            while(arm_extendThread.isAlive()) idle();
        }
    }

    /*********************** Thread Classes ********************/
    private class routineTaskThread extends Thread        {
        public routineTaskThread()
        {
            this.setName("routineTaskThread");
        }
        @Override
        public void run()
        { // runs routine task at 50ms cycles
            try
            {
                while (!isInterrupted()  && !isStopRequested())
                {
                    //localization();
                    telemetry.addData("Pan: ",getArmPanPosition());
                    telemetry.addData("Lift: ",getArmLiftPosition());
                    telemetry.addData("extend: ",getArmExtendPosition());

                    telemetry.addData("Servo Tilt", getArmWristTiltPosition());
                    telemetry.addData("Servo Flip", getArmWristFlipPosition());
                    telemetry.update();
                    //sleep(10);
                    idle();

                }
            }
            // an error occurred in the run loop.
            catch (Exception e) {}
        }
    }
    private class Arm_Lift_PositionThread extends Thread        {
        int   delay_start;
        double target, power;
        public Arm_Lift_PositionThread(double target, double power, int delay_start)
        {
            this.setName("Arm_Lift_PositionThread");
            this.target = target;
            this.power = power;
            this.delay_start = delay_start;
        }
        @Override
        public void run()
        {
            try
            {
                sleep(delay_start);
                Arm_Lift_Position(target, power);
            }
            // an error occurred in the run loop.
            catch (Exception e) {;}
        }
    }
    private class Arm_Wrist_PositionThread extends Thread        {
        int target,  delay_start;
        public Arm_Wrist_PositionThread(int target, int delay_start)
        {
            this.setName("Arm_Wrist_PositionThread");
            this.target = target;
            this.delay_start = delay_start;
        }
        @Override
        public void run()
        {
            try
            {
                sleep(delay_start);
                wrist_Tilt_Servo(target);
            }
            // an error occurred in the run loop.
            catch (Exception e) {;}
        }
    }
    private class Arm_Pan_PositionThread extends Thread        {
        int delayMS;
        double target, power;
        public Arm_Pan_PositionThread(double target, double power, int delayMS)
        {
            this.setName("Arm_Pan_PositionThread");
            this.target = target;
            this.power = power;
            this.delayMS = delayMS;
        }
        @Override
        public void run()
        {
            try
            {
                sleep(delayMS);
                Arm_Pan_position(target, power);
            }
            // an error occurred in the run loop.
            catch (Exception e) {;}
        }
    }
    private class Arm_Extend_PositionThread extends Thread        {
        double target;
        int  delayMS;
        double power;
        public Arm_Extend_PositionThread(double target, double power, int delayMS)
        {
            this.setName("Arm_Extend_PositionThread");
            this.target = target;
            this.power = power;
            this.delayMS = delayMS;
        }
        @Override
        public void run()
        {
            try
            {
                sleep(delayMS);
                Arm_Extend_Position(target, power);
            }
            // an error occurred in the run loop.
            catch (Exception e) {}
        }
    }

    private class goXYnTurnThread extends Thread        {
        double heading, targetX, targetY, V_max, Tracking_resolution, V_max_Turn,turn_resolution;
        boolean isLockPosition;
        public boolean isgoXYnTurn_finished = false;
        int delayMS;
        boolean isBrakeOn;
        int wallEnable;

        public goXYnTurnThread( double heading, double targetX, double targetY, double V_max,
                                double Tracking_resolution, double V_max_Turn, double turn_resolution, int wallEnable, boolean isLockPosition, boolean isBrakeOn,  int delayMS)
        {
            this.heading = heading;
            this.targetX= targetX;          this.targetY= targetY;
            this.V_max= V_max;                this.Tracking_resolution= Tracking_resolution;
            this.V_max_Turn = V_max_Turn;         this.turn_resolution = turn_resolution;
            this.isLockPosition = isLockPosition;
            this.delayMS = delayMS;
            this.isBrakeOn = isBrakeOn;
            this.wallEnable = wallEnable;
        }
        @Override
        public void run()
        {
            try
            {
                sleep(delayMS);
                isgoXYnTurn_finished =  goXYnTurn(heading, targetX, targetY, V_max, Tracking_resolution, V_max_Turn, turn_resolution, wallEnable, isLockPosition, isBrakeOn);
            }
            catch (Exception e) {;}
        }
    }

    /********************** Runnable Thread *******************************/

    void telePickupFromSubStation( ){
        new Thread(new Runnable() {

            @Override
            public void run() {

                isAutoEnable = true;
                coneLiftingMissionCompleted = false;
                conePickupMissionCompleted = false;

                claw_Open();
                arm_extendThread = new Arm_Extend_PositionThread(0.1, 0.9, 0);
                arm_extendThread.start();
                arm_liftThread = new Arm_Lift_PositionThread( conePickupLiftAngle,0.6, 200);
                arm_liftThread.start();
                arm_panThread = new Arm_Pan_PositionThread(conePickupPanAngle, 0.4,200);
                arm_panThread.start();
                while (arm_extendThread.isAlive()) idle();
                wrist_Flip_Servo(-90);
                wrist_Tilt_Servo(-45);
                while (arm_extendThread.isAlive() || arm_liftThread.isAlive() ) idle();
                wrist_cone_pickup_back_ready(0);
                arm_extendThread = new Arm_Extend_PositionThread(conePickupExtendLength, 0.7, 0);
                arm_extendThread.start();

                while (arm_extendThread.isAlive()) idle();
                claw_Close();
                sleep(200);
                wrist_cone_pickup_back_Up(0);
                conePickupMissionCompleted = true;
                isInAuto = false;
            }
        }).start();
    }
    void teleConeToHighPoleTask(){
        new Thread(new Runnable() {
            @Override
            public void run() {
                isAutoEnable = true;
                coneLiftingMissionCompleted = false;
                conePickupMissionCompleted = false;


                claw_Close();
                arm_extendThread = new Arm_Extend_PositionThread(0.16, 0.9, 0);
                arm_extendThread.start();
                arm_liftThread = new Arm_Lift_PositionThread( coneDropLiftAngle,0.75, 0);
                arm_liftThread.start();
                arm_panThread = new Arm_Pan_PositionThread(coneDropPanAngle, 0.4,150);
                arm_panThread.start();

                sleep(500);
                wrist_Tilt_Servo(-45);
                wrist_Flip_Servo(90);

                arm_extendThread = new Arm_Extend_PositionThread(coneDropExtendLength, 0.7, 0);
                arm_extendThread.start();
                while  (arm_liftThread.isAlive()) idle();
                wrist_cone_front_ready(0);

                //Wait until it reach the destination

                while (!isStopRequested() &&(arm_panThread.isAlive()|| arm_liftThread.isAlive() || arm_extendThread.isAlive()) ){
                    if (!isAutoEnable)
                    {//to stop all thread tasks above
                        Arm_pan_motorEnabled = false;
                        Arm_lift_motorEnabled = false;
                        Arm_extend_motorEnabled = false;
                    }
                    idle();
                }
                coneLiftingMissionCompleted = true;
                //wrist_cone_front_drop(0);
                isInAuto = false;
            }
        }).start();
    }


    void armUpRightPositionTask(){
        new Thread(new Runnable() {
            @Override
            public void run() {
                isAutoEnable = true;
                arm_extendThread = new Arm_Extend_PositionThread(0.01, 0.95, 0);
                arm_extendThread.start();
                arm_liftThread = new Arm_Lift_PositionThread( -10,0.95, 0);
                arm_liftThread.start();
                arm_panThread = new Arm_Pan_PositionThread(0, 0.6,0);
                arm_panThread.start();
                sleep(300);
                wrist_Tilt_Servo(-45);
                while (!isStopRequested() &&(arm_panThread.isAlive()|| arm_liftThread.isAlive() || arm_extendThread.isAlive()) )
                {
                    if (!isAutoEnable)
                    {//to stop all thread tasks above
                        Arm_extend_motorEnabled = false;
                        Arm_lift_motorEnabled = false;
                        Arm_pan_motorEnabled = false;
                    }
                    idle();
                }
                isInAuto = false;
            }
        }).start();
    }
    private class ScanConeThread extends Thread
    {
        public boolean runningFlag = false;
        public boolean resetFlag = false;

        private double leftDis = 2000;
        private double rightDis = 2000;
        private final double DETECT_DIS = 150; // 50cm = 0.5m

        private double scanAngle = 0;
        private boolean foundCone = false;
        public ScanConeThread()
        {
            scanAngle = getArmPanPosition();
            runningFlag = true;
            foundCone = false;
            scanFlag = true;

        }

        @Override
        public void run()
        {
            double ctrlValue = 3;
            while(opModeIsActive() && runningFlag)
            {
                leftDis = leftd.getDistance(DistanceUnit.MM);
                rightDis = rightd.getDistance(DistanceUnit.MM);

                if ((leftDis < DETECT_DIS || rightDis < DETECT_DIS)&&(leftDis - rightDis > 10)){
                    foundCone = true;
                }
                else{
                    scanAngle += ctrlValue;
                    Arm_PanMotor.setTargetPosition((int) (scanAngle * turret_Pulse_per_Degree));
                    Arm_PanMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm_PanMotor.setPower(scanAngle);
                    foundCone = false;
                }
                telemetry.addData("hLeftDis", "%.2f", leftd.getDistance(DistanceUnit.MM));
                telemetry.addData("hRightDis", "%.2f", rightd.getDistance(DistanceUnit.MM));
                telemetry.addData("cone detected",foundCone);

                telemetry.update();
            }
            isInAuto = false;

        }

        public boolean isFoundCone(){
            return foundCone;
        }
        public void stopScan(){
            runningFlag = false;
            scanFlag = false;
        }
    }


    private void armUpRightPosition()
    {
        arm_extendThread = new Arm_Extend_PositionThread(0.01, 0.95, 0);
        arm_extendThread.start();
        arm_liftThread = new Arm_Lift_PositionThread( -10,0.95, 0);
        arm_liftThread.start();
        arm_panThread = new Arm_Pan_PositionThread(0, 0.6,0);
        arm_panThread.start();
        sleep(300);
        wrist_Tilt_Servo(-45);
    }

    private void stopAllMotors(){
        brakeAll(false, true);
        Arm_PanMotor.setPower(0);
        Arm_lift_Left_Motor.setPower(0);
        Arm_lift_Right_Motor.setPower(0);
        Arm_ExtendMotor.setPower(0);
    }
    private class ScanJunctionThread extends Thread {
        public boolean runningFlag = false;
        public boolean resetFlag = false;

        private double leftDis = 2000;
        private double rightDis = 2000;
        private double middleDis = 2000;
        private final double DETECT_DIS = 250; // 50cm = 0.5m

        private double scanAngle = 0;
        private boolean foundJunction = false;
        public int scanlowrange;
        public int scanhighrange;

        public ScanJunctionThread(int low, int high) {
            scanAngle = getArmPanPosition();
            scanlowrange = low;
            scanhighrange = high;
            runningFlag = true;
            foundJunction = false;
            scanFlag = true;
        }

        @Override
        public void run() {
            scanAngle = getArmPanPosition();
            double scanStep = 2;
            boolean scanDir = true;

            double scanHigh = scanAngle + 15;
            scanHigh = scanHigh > scanhighrange ? scanhighrange : scanHigh;

            double scanLow = scanAngle - 15;
            scanLow = scanLow < scanlowrange ? scanlowrange : scanLow;

            while (opModeIsActive() && runningFlag && !foundJunction) {
                leftDis = leftd.getDistance(DistanceUnit.MM) - 27;
                middleDis = centerd.getDistance(DistanceUnit.MM) - 22;
                rightDis = rightd.getDistance(DistanceUnit.MM) - 24;

                if (middleDis < leftDis && middleDis < rightDis && middleDis < DETECT_DIS) {
                    foundJunction = true;
                }
                else if (leftDis < middleDis && leftDis < DETECT_DIS) {
                    scanAngle = scanAngle + 3;
                    arm_panThread = new Arm_Pan_PositionThread(scanAngle, 0.6, 0);
                    arm_panThread.start();
                    while (arm_panThread.isAlive()) idle();
                    foundJunction = true;
                }
                else if (rightDis < middleDis && rightDis < DETECT_DIS) {
                    scanAngle = scanAngle - 3;
                    arm_panThread = new Arm_Pan_PositionThread(scanAngle, 0.6, 0);
                    arm_panThread.start();
                    while (arm_panThread.isAlive()) idle();
                    foundJunction = true;
                }
                else{
                    if (scanDir) {
                        scanAngle += scanStep;
                        if (scanAngle > scanHigh) {
                            telemetry.addData("off left","");
                            telemetry.update();
                            scanAngle = 60;
                            scanDir = false;
                        }
                        arm_panThread = new Arm_Pan_PositionThread(scanAngle, 0.6, 0);
                        arm_panThread.start();
                        while (arm_panThread.isAlive()) idle();
                    } else {
                        scanAngle -= scanStep;
                        if (scanAngle < scanLow) {
                            telemetry.addData("off right","");
                            telemetry.update();
                            scanAngle = 35;
                            scanDir = true;
                        }
                        arm_panThread = new Arm_Pan_PositionThread(scanAngle, 0.6, 0);
                        arm_panThread.start();
                        while (arm_panThread.isAlive()) idle();
                    }

                }



            }

            telemetry.addData("Angle", scanAngle);
            telemetry.addData("right:", String.format("%.01f mm", rightd.getDistance(DistanceUnit.MM)));
            telemetry.addData("left:", String.format("%.01f mm", leftd.getDistance(DistanceUnit.MM)));
            telemetry.addData("center:", String.format("%.01f mm", centerd.getDistance(DistanceUnit.MM)));
            telemetry.update();

        }


        public boolean isFoundJunction() {
            return foundJunction;
        }

        public void stopScan() {
            runningFlag = false;
            scanFlag = false;
        }
    }
}
