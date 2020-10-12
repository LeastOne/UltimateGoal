//Was halfway through commenting on the math to make sure I understood it but never finished.

package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class MecanumController extends RobotController {

    public MecanumController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if(robot.mecanumMode){
            //since left stick can be pushed in all directions to control the robot's movements, its "power" must be the actual
            //distance from the center, or the hypotenuse of the right triangle formed by left_stick_x and left_stick_y
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            //angle between x axis and "coordinates" of left stick
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;

            //turn
            double rightX = gamepad1.right_stick_x;

            double lf = r * Math.cos(robotAngle) + rightX;;
            double lb = r * Math.sin(robotAngle) + rightX;;
            double rf = r * Math.sin(robotAngle) - rightX;;
            double rb = r * Math.cos(robotAngle) - rightX;;

            robot.drive(lf,lb,rf,rb);
        }
        else{
            if(gamepad1.left_stick_button&&gamepad1.right_stick_button){
                robot.mecanumMode=true;
            }
        }

    }
}
