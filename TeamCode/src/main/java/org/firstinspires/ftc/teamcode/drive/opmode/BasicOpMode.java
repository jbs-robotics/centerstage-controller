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

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOpMode extends LinearOpMode {

    // Declare OpMode members.
    private int pullupUp = 12690, pullupDown = 0, counter = 0, counter2 = 0;
    ;
    private double servoUpLimit = 0.75;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, leftBack, rightFront, rightBack, lift, pullupMotor, urchin = null;
    private Servo intakeServo = null, droneLauncher = null, claw = null;
    private CRServo hookServo = null, angleServo = null;
    private double currentServoPos = 0.75, sensitivity = 1, driveSensitivity = .75, brakingOffset = -0.1, angleServoPos = 0.45, clawPos = 0.43;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Movement Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Lift Motors
        lift = hardwareMap.get(DcMotor.class, "lift");
        angleServo = hardwareMap.get(CRServo.class, "angleServo");
        claw = hardwareMap.get(Servo.class, "claw");
//        urchin = hardwareMap.get(DcMotor.class, "urchin");

        // Pullup Motors
        pullupMotor = hardwareMap.get(DcMotor.class, "pullup");
//        hookServo = hardwareMap.get(CRServo.class, "hookServo");
        intakeServo = hardwareMap.get(Servo.class, "intake");

        // Launcher Motors
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");


        // To drive forward, most robots need the motor on on e side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //clockwise = forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        pullupMotor.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        //resets the encoders 0 position
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//        pullupMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pullupMotor.setTargetPosition(0);

        pullupMotor.setPower(1);
        pullupMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeServo.setPosition(servoUpLimit);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            boolean sniperModeOn = gamepad2.left_bumper;
            boolean sniperModeOff = gamepad2.right_bumper;
            double liftControl = gamepad2.left_stick_y;
            double intake = -gamepad2.right_stick_y;
            boolean hookBtn = gamepad2.y;
            //a and b are switched on gamepad
            boolean lockOn = gamepad2.b;
            boolean lockOff = gamepad2.a;
            boolean driveSnipeOn = gamepad1.left_bumper;
            boolean driveSnipeOff = gamepad1.right_bumper;
            //gamepad 1(drivebase control)
            double lfPower = Range.clip(drive + turn + strafe, -driveSensitivity, driveSensitivity) ;
            double rfPower = Range.clip(drive - turn - strafe, -driveSensitivity, driveSensitivity) ;
            double lbPower = Range.clip(drive + turn - strafe, -driveSensitivity, driveSensitivity);
            double rbPower = Range.clip(drive - turn + strafe, -driveSensitivity, driveSensitivity) ;

            //gamepad 2(lift control)
            double intakePos = Range.clip(intake, -sensitivity/180, sensitivity/180);
            double liftPower = Range.clip(liftControl, -sensitivity, sensitivity);
            boolean launch = gamepad2.dpad_up;
            boolean unlaunch = gamepad2.dpad_down;
            boolean angleServoUp = gamepad1.dpad_up, angleServoDown = gamepad1.dpad_down, clawUp = gamepad1.dpad_right, clawDown = gamepad1.dpad_left;


            // Send calculated power to wheels
            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);

            //send power to lift
            if(liftPower == 0) liftPower = brakingOffset;
            lift.setPower(liftPower);

            //check if intake is running
            currentServoPos += intakePos;
            currentServoPos = Math.max(0, currentServoPos);
            currentServoPos = Math.min(.75, currentServoPos);
            intakeServo.setPosition(currentServoPos);
            if (sniperModeOff) sensitivity = 1;
            if (sniperModeOn) sensitivity = 0.5;
            if (driveSnipeOn) driveSensitivity = 0.25;
            if (driveSnipeOff) driveSensitivity = .75;
            if (hookBtn) {
                pullupMotor.setTargetPosition(pullupUp);
                pullupMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pullupMotor.setPower(1);
            }
//            angleServo.setPosition(angleServoPos);
            claw.setPosition(clawPos);
            if(angleServoUp) {
                angleServo.setPower(.2);
            }
            else if(angleServoDown) {
                angleServo.setPower(-.2);
            }
            else angleServo.setPower(0);
            if(clawUp && clawPos <= 1) clawPos += 0.001;
            if(clawDown && clawPos >= 0) clawPos -= 0.001;
            if(gamepad2.x){
                pullupMotor.setTargetPosition(pullupDown);
                pullupMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pullupMotor.setPower(1);
            }
            if (launch){
                droneLauncher.setPosition(0);
            }
            if (unlaunch){
                droneLauncher.setPosition(0.5);
            }
            telemetry.addData("Current Intake Servo Pos: ", currentServoPos);
            telemetry.addData("Sensitivity: ", sensitivity);
            telemetry.addData("Drive Sensitivity: ", driveSensitivity);
            telemetry.addData("pullupMotor Position: ", pullupMotor.getCurrentPosition());
            telemetry.addData("pullupMotor Target Position: ", pullupMotor.getTargetPosition());
            telemetry.addData("LiftPower: ", lift.getPower());
            telemetry.addData("angleServoPos: ", angleServo.getPower());
            telemetry.addData("clawPos", claw.getPosition());
            telemetry.addData("counter: ", counter);
            telemetry.addData("counter2: ", counter2);
            telemetry.update();
        }
    }
}
