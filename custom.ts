
/**
 * Use this file to define custom functions and blocks.
 * Read more at https://makecode.microbit.org/blocks/custom
 */

enum gigglebotWhichUniqueMotor {
    //% block="right motor"
    Right,
    //% block="left motor"
    Left
}

enum gigglebotWhichMotor {
    //% block="both motors"
    Both,
    //% block="right motor"
    Right,
    //% block="left motor"
    Left

}

enum gigglebotWhichDriveDirection {
    //% block="forward"
    Forward,
    //% block="backward"
    Backward
}

enum gigglebotWhichTurnDirection {
    //% block="right"
    Right,
    //% block="left"
    Left
}

enum gigglebotWhichUnitSystem {
    //% block="mm"
    mm,
    //% block="inches"
    inches
}

enum gigglebotWhichSpeed {
    //% block="slowest"
    Slowest = 30,
    //% block="slower"
    Slower = 45,
    //% block="normal"
    Normal = 60,
    //% block="faster"
    Faster = 75,
    //% block="fastest"
    Fastest = 90
}

enum gigglebotI2CCommands {
    GET_FIRMWARE_VERSION = 1,
    GET_MANUFACTURER,
    GET_BOARD,
    GET_VOLTAGE_BATTERY,
    GET_LINE_SENSORS,
    GET_LIGHT_SENSORS,
    GET_MOTOR_STATUS_RIGHT,
    GET_MOTOR_STATUS_LEFT,
    SET_MOTOR_POWER,
    SET_MOTOR_POWERS
}

/**
* Custom blocks
*/



//% weight=99 color=#46BFB1 icon="\uf0d1"
//% groups='["other", "Line Follower", "Light Sensors", "Servo", "Distance Sensor (Add-On)", "Temperature Humidity Pressure (Add-On)", "On Board Sensors", "Voltage", "Firmware"]'
namespace gigglebot {
    /**
     * Basic drive and sensor functionalities for GiggleBot
     * No radio, no neopixels functionalities here in order to be compatible with Bluetooth.
     * Load pxt-giggle for radio and neopixels
     */
    let ADDR = 0x04
    let line_follower_threshold = 175
    let light_level_threshold = 850
    let currentMotorPower = gigglebotWhichSpeed.Normal;
    let trimLeft = 0
    let trimRight = 0
    let motorPowerLeft = currentMotorPower
    let motorPowerRight = currentMotorPower
    let distanceSensorInitDone = false;
    let thpSensorInitDone = false;
    let line_follow_in_action = false;
    let light_follow_in_action = false;
    let lineSensors = [0, 0]
    let lightSensors = [0, 0]
    // turn motor power off
    stop()


    /**
     * return current power setting of the left motor
     */
    export function leftPower() {
        return motorPowerLeft
    }

    /**
     * return current power setting of the right motor
     */
    export function rightPower() {
        return motorPowerRight
    }

    /**
     * Assigns a new power value to the left motor
     * Values from 101 through 127, and -128 through -101 are used to float the  motor.
     * @param leftpower new value for the power setting of the left motor (-100 < leftpower < 100)
     */
    export function setLeftPower(leftpower: number) {
        motorPowerLeft = leftpower
    }
    /**
     * Assigns a new power value to the right motor
     * Values from 101 through 127, and -128 through -101 are used to float the  motor.
     * @param rightpower new value for the power setting of the right motor. (-100 < rightpower < 100)
     */
    //% rightpower.min= -100 rightpower.max = 100
    export function setRightPower(rightpower: number) {
        motorPowerRight = rightpower
    }


    ////////////////////////////////////////////////////////////////////////
    ////////// BLOCKS
    ///////////////////////////////////////////////////////////////////////

    /**
     * Will let GiggleBot move forward or backward for a number of milliseconds.
     * Distance covered during that time is related to the freshness of the batteries.
     * @param dir forward or backward; 
     * @param delay for how many milliseconds; eg: 1000
     */
    //% blockId="gigglebotDriveMillisec" block="drive %dir|for %delay|ms"
    //% weight=100
    //% delay.min=0
    export function driveMillisec(dir: gigglebotWhichDriveDirection, delay: number) {
        if (delay < 0) delay = 0
        driveStraight(dir)
        basic.pause(delay)
        stop()
    }

    /**
     * Will make GiggleBot turn left and right for a number of milliseconds. How far it turns depends on the freshness of the batteries.
     * @param turn_dir turning left or right
     * @param delay for how many milliseconds; eg: 1000
     */
    //% blockId="gigglebotTurnMillisec" block="turn %turn_dir|for %delay|ms"
    //% weight=99
    //% delay.min=0
    export function turnMillisec(turn_dir: gigglebotWhichTurnDirection, delay: number) {
        if (delay < 0) delay = 0
        turn(turn_dir)
        basic.pause(delay)
        stop()
    }

    /** 
     * GiggleBot will spin on itself for the provided number of milliseconds, like a turn but staying in the same spot. Especially useful when drawing
     * @param turn_dir turning left or right
     * @param delay how many milliseconds; eg: 1000
     */
    //% blockId="gigglebotSpinMillisec" block="spin %turn_dir|for %delay|ms"
    //% weight=98
    //% delay.min=0
    export function spinMillisec(turn_dir: gigglebotWhichTurnDirection, delay: number) {
        if (delay < 0) delay = 0
        gigglebotSpin(turn_dir)
        basic.pause(delay)
        stop()
    }

    /** 
     * GiggleBot will drive forward while steering to one side for the provided number of milliseconds. 
     * Useful when it needs to go around an obstacle, or orbit around an object.
     * 0% means no steering, the same as the 'drive' block. 100% is the same as the 'turn' block.
     * @param percent the variation in power between left and right; eg: 0, 20, 50, 100
     * @param dir which direction to steer, left or right
     * @param delay for how many milliseconds; eg: 1000
     *      */
    //% blockId="gigglebotSteerMillisec" block="steer %percent| towards the %dir| for %delay| ms"
    //% percent.min=0 percent.max=100
    //% weight=97
    export function steerMillisec(percent: number, dir: gigglebotWhichTurnDirection, delay: number) {
        if (delay < 0) delay = 0
        if (percent < 0) percent = 0
        if (percent > 100) percent = 100
        steer(percent, dir)
        basic.pause(delay)
        stop()
    }

    /**
     * Will let GiggleBot move forward or backward until told otherwise (either by a stop block or a turn block).
     * @param dir forward or backward
     */
    //% blockId="gigglebot_drive_straight" block="drive %dir"
    //% weight=89
    export function driveStraight(dir: gigglebotWhichDriveDirection) {
        let dir_factor = 1
        if (dir == gigglebotWhichDriveDirection.Backward) {
            dir_factor = -1
        }
        if (dir == gigglebotWhichDriveDirection.Forward) {
            dir_factor = 1
        }
        motorPowerAssignBoth(motorPowerLeft * dir_factor, motorPowerRight * dir_factor)
    }

    /**
     * Will make GiggleBot turn left or right until told otherwise (by a stop block or a drive block).
     */
    //% blockId="gigglebotTurn" block="turn %turn_dir"
    //% weight=88
    export function turn(turn_dir: gigglebotWhichTurnDirection) {
        if (turn_dir == gigglebotWhichTurnDirection.Left) {
            motorPowerAssignBoth(0, motorPowerRight)
        }
        else {
            motorPowerAssignBoth(motorPowerLeft, 0)
        }
    }

    /** 
     * GiggleBot will spin on itself until told otherwise, like a turn but staying in the same spot. Especially useful when drawing.
     * @param turn_dir left or right;
     */
    //% blockId="gigglebotSpin" block="spin %turn_dir"
    //% weight=87
    export function gigglebotSpin(turn_dir: gigglebotWhichTurnDirection) {
        if (turn_dir == gigglebotWhichTurnDirection.Left) {
            motorPowerAssignBoth(-1 * motorPowerLeft, motorPowerRight)
        }
        else {
            motorPowerAssignBoth(motorPowerLeft, -1 * motorPowerRight)
        }
    }

    /** 
     * GiggleBot will drive forward while steering to one side. 
     * Useful when it needs to go around an obstacle, or orbit around an object.
     * 0% means no steering, the same as the 'drive' block. 100% is the same as the 'turn' block.
     * @param percent value between 0 and 100 to control the amount of steering
     * @param dir to the left or to the right
     */
    //% blockId="gigglebotSteer" block="steer %percent| towards the %dir"
    //% percent.min=0 percent.max=100
    //% weight=86
    export function steer(percent: number, dir: gigglebotWhichTurnDirection) {
        percent = Math.min(Math.max(percent, 0), 100)
        let correctedMotorPowerLeft = motorPowerLeft
        let correctedMotorPowerRight = motorPowerRight
        if (dir == gigglebotWhichTurnDirection.Left) {
            correctedMotorPowerLeft = motorPowerLeft - Math.idiv(motorPowerLeft * percent, 100)
            correctedMotorPowerRight = motorPowerRight + Math.idiv(motorPowerRight * percent, 100)
        } else {
            correctedMotorPowerLeft = motorPowerLeft + Math.idiv(motorPowerLeft * percent, 100)
            correctedMotorPowerRight = motorPowerRight - Math.idiv(motorPowerRight * percent, 100)
        }
        motorPowerAssignBoth(correctedMotorPowerLeft, correctedMotorPowerRight)
    }

    /**
    * stops the robot.
    */
    //% blockId="gigglebot_stop" block="stop"
    //% weight=70
    export function stop() {
        motorPowerAssign(gigglebotWhichMotor.Both, 0)
        light_follow_in_action = false
        line_follow_in_action = false
    }

    /**
     * You can set the speed for each individual motor or both together. The higher the speed the less control the robot has.
     * You may need to correct the robot (see block in "more..." section).  A faster robot needs more correction than a slower one.
     * Note that any drive correction done previously gets applied here.
     * If you want to follow a line,  it will work best at a lower speed.
     * Actual speed is dependent on the freshness of the batteries.
     * @param motor: left, right or both motors
     * @param speed: how fast the robot goes.
     */
    //% blockId="gigglebot_set_speed" block="set %motor | speed to %speed"
    //% speed.min=-100 speed.max=100
    //% weight=60
    export function setSpeed(motor: gigglebotWhichMotor, speed: gigglebotWhichSpeed) {
        speed = Math.min(Math.max(speed, -100), 100)
        currentMotorPower = speed
        motorPowerLeft = currentMotorPower
        motorPowerRight = currentMotorPower

        // apply trim 
        if (trimRight != 0 && motor != gigglebotWhichMotor.Left) {
            if (speed > 0) {
                motorPowerRight = currentMotorPower - Math.idiv(trimRight * currentMotorPower, 100);
            } else {
                motorPowerRight = currentMotorPower + Math.idiv(trimRight * currentMotorPower, 100);
            }
        }
        if (trimLeft != 0 && motor != gigglebotWhichMotor.Right) {
            if (speed > 0) {
                motorPowerLeft = currentMotorPower - Math.idiv(trimLeft * currentMotorPower, 100);
            } else {
                motorPowerLeft = currentMotorPower + Math.idiv(trimLeft * currentMotorPower, 100);
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////
    /////////// MORE BLOCKS
    ///////////////////////////////////////////////////////////////////////


    /**
     * This allows the user to correct the motors on the GiggleBot if it's not driving straight
     * @param dir: if the GiggleBot drives to the left, then correct to the right. Vice versa. 
     * @param trim_value: a correction value between 0 and 100, but most likely below 10
     */
    //% blockId="gigglebot_trim" block="correct towards %dir|by %trim_value"
    //% weight=100
    //% advanced=true
    export function motorTrimSet(dir: gigglebotWhichTurnDirection, trim_value: number) {
        if (trim_value < 0) {
            trim_value = 0
        }
        if (dir == gigglebotWhichTurnDirection.Left) {
            trimLeft = trim_value
            trimRight = 0
        }
        if (dir == gigglebotWhichTurnDirection.Right) {
            trimRight = trim_value
            trimLeft = 0
        }
        if (motorPowerLeft > 0) {
            motorPowerLeft = currentMotorPower - Math.idiv(trimLeft * currentMotorPower, 100)
        } else {
            motorPowerLeft = currentMotorPower + Math.idiv(trimLeft * currentMotorPower, 100)
        }
        if (motorPowerRight > 0) {
            motorPowerRight = currentMotorPower - Math.idiv(trimRight * currentMotorPower, 100)
        } else {
            motorPowerRight = currentMotorPower + Math.idiv(trimRight * currentMotorPower, 100)
        }
    }

    /** 
     * Assigns power to a motor, or the same power to both motors
     * Values from 101 through 127, and -128 through -101 are used to float the  motor.
     * @param motor:  left or right motor, or both
     * @param power: a value between -100 and 100
     */
    //% blockId="gigglebot_set_motor" block="set power on %motor| to | %power"
    //% advanced=true
    //% weight=90
    export function motorPowerAssign(motor: gigglebotWhichMotor, power: number) {
        let buf = pins.createBuffer(3)
        buf.setNumber(NumberFormat.UInt8BE, 0, gigglebotI2CCommands.SET_MOTOR_POWER)
        buf.setNumber(NumberFormat.UInt8BE, 2, power)
        // activate right motor
        if (motor == gigglebotWhichMotor.Right) {
            buf.setNumber(NumberFormat.UInt8BE, 1, 0x01)
        }
        // activate left motor
        else if (motor == gigglebotWhichMotor.Left) {
            buf.setNumber(NumberFormat.UInt8BE, 1, 0x02)
        }
        // activate both motors
        else if (motor == gigglebotWhichMotor.Both) {
            buf.setNumber(NumberFormat.UInt8BE, 1, 0x03)
        }
        pins.i2cWriteBuffer(ADDR, buf, false);
    }

    /**
     * Assigns potentially different powers to both motors in one call.  
     * Values from 101 through 127, and -128 through -101 are used to float the  motor.
     * @param left_power: the power to assign to the left motor (between -100 and 100)
     * @param right_power: the power to assign to the right motor (between -100 and 100)
     */
    //% blockId="gigglebot_set_motors" block="set left power to %left_power|and right to | %right_power"
    //% advanced=true
    //% weight=90
    export function motorPowerAssignBoth(left_power: number, right_power: number) {
        let buf = pins.createBuffer(3)
        buf.setNumber(NumberFormat.UInt8BE, 0, gigglebotI2CCommands.SET_MOTOR_POWERS)
        buf.setNumber(NumberFormat.UInt8BE, 1, right_power)
        buf.setNumber(NumberFormat.UInt8BE, 2, left_power)
        pins.i2cWriteBuffer(ADDR, buf, false);
    }

}
