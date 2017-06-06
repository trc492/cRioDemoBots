#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="DriveBase.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     DriveBase class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _DRIVEBASE_H
#define _DRIVEBASE_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_DRIVEBASE
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "DriveBase"

/**
 *  This abstract class defines the MotorPosition object. The object is a
 *  callback interface. It is not meant to be created as an object. Instead,
 *  it should be inherited by a subclass who needs to provide motor position
 *  data for the DriveBase to compute the robot position. This is typical by
 *  reading the encoders.
 */
class MotorPosition
{
public:
    /**
     *  This function is provided by the subclass to return the motor position.
     *  It is typically by reading the encoder position.
     *
     *  @param motor Specifies the motor controller for reading its position.
     *
     *  @return Returns the motor position in RPM.
     */
    virtual
    float
    GetMotorPosition(
        SpeedController *motor
        ) = 0;
};  //class MotorPosition

/**
 *  This class defines and implements the DriveBase object. It supports three
 *  different drive mechanisms: Arcade, Tank and Mecanum. The arcade and tank
 *  drive mechanisms support both 2-motor or 4-motor configurations. For
 *  mecanum drive, it must have 4 motors. Aside from providing the methods
 *  for different drive mechanisms, this object also keeps track of the robot
 *  position/heading by using encoder on each motor and a gyro.
 */

#define MAX_NUM_MOTORS          4

#define IDX_LEFT_FRONT          0
#define IDX_LEFT_REAR           1
#define IDX_RIGHT_FRONT         2
#define IDX_RIGHT_REAR          3

#define DRIVEF_FOUR_MOTORS      0x00000001
#define DRIVEF_MECANUM          0x00000002

#ifdef _DEBUG_DRIVEBASE
#define VARID_XDRIVE_PID        (VARID_DATAPTR + 1)
#define VARID_XDRIVE_KP         (VARID_DATAPTR + 2)
#define VARID_XDRIVE_KI         (VARID_DATAPTR + 3)
#define VARID_XDRIVE_KD         (VARID_DATAPTR + 4)
#define VARID_XDRIVE_KF         (VARID_DATAPTR + 5)
#define VARID_YDRIVE_PID        (VARID_DATAPTR + 6)
#define VARID_YDRIVE_KP         (VARID_DATAPTR + 7)
#define VARID_YDRIVE_KI         (VARID_DATAPTR + 8)
#define VARID_YDRIVE_KD         (VARID_DATAPTR + 9)
#define VARID_YDRIVE_KF         (VARID_DATAPTR + 10)
#define VARID_TURN_PID          (VARID_DATAPTR + 11)
#define VARID_TURN_KP           (VARID_DATAPTR + 12)
#define VARID_TURN_KI           (VARID_DATAPTR + 13)
#define VARID_TURN_KD           (VARID_DATAPTR + 14)
#define VARID_TURN_KF           (VARID_DATAPTR + 15)
#endif

class DriveBase
    : public CoopTask
    , public RobotDrive
    , public PIDInput
#ifdef _DEBUG_DRIVEBASE
    , public CmdHandler
#endif
{
private:
    Gyro               *m_gyro;
    SpeedController    *m_motors[MAX_NUM_MOTORS];
    TrcPIDCtrl         *m_xPidCtrl;
    TrcPIDCtrl         *m_yPidCtrl;
    TrcPIDCtrl         *m_turnPidCtrl;
    MotorPosition      *m_motorPosition;
    TrcPIDDrive         m_pidDrive;
    int                 m_encPolarities[MAX_NUM_MOTORS];
    float               m_startPos[MAX_NUM_MOTORS];
    float               m_xDistPerRev;
    float               m_yDistPerRev;
    float               m_degPerRev;
    float               m_xPos;
    float               m_yPos;
    float               m_rotPos;
    float               m_heading;
    UINT32              m_driveBaseFlags;

    /**
     *  This function is called from different constructors to do common
     *  initialization of the object class.
     *
     *  @param leftFrontMotor Specifies the left front motor controller.
     *  @param leftRearMotor Specifies the left rear motor controller.
     *  @param rightFrontMotor Specifies the right front motor controller.
     *  @param rightRearMotor Specifies the right rear motor controller.
     *  @param xPidCtrl Specifies the PID controller for sideway.
     *  @param yPidCtrl Specifies the PID controller for forward/backward.
     *  @param turnPidCtrl Specifies the PID controller for turning.
     *  @param gyro Specifies the gyro used to keep track of the heading.
     *  @param motorPosition Specifies the MotorPosition provider object.
     */
    void
    CommonInit(
        SpeedController *leftFrontMotor,
        SpeedController *leftRearMotor,
        SpeedController *rightFrontMotor,
        SpeedController *rightRearMotor,
        TrcPIDCtrl      *xPidCtrl,
        TrcPIDCtrl      *yPidCtrl,
        TrcPIDCtrl      *turnPidCtrl,
        Gyro            *gyro,
        MotorPosition   *motorPosition
        )
    {
        TLevel(INIT);
        TEnterMsg(("lf=%p,lr=%p,rf=%p,rr=%p,xPid=%p,yPid=%p,turnPid=%p,gyro=%p,motorPosition=%p",
                   leftFrontMotor, leftRearMotor, rightFrontMotor,
                   rightRearMotor, xPidCtrl, yPidCtrl, turnPidCtrl,
                   gyro, motorPosition));

        for (int i = 0; i < MAX_NUM_MOTORS; i++)
        {
            m_encPolarities[i] = 1;
            m_startPos[i] = 0.0;
        }

        m_motors[IDX_LEFT_FRONT] = leftFrontMotor;
        m_motors[IDX_LEFT_REAR] = leftRearMotor;
        m_motors[IDX_RIGHT_FRONT] = rightFrontMotor;
        m_motors[IDX_RIGHT_REAR] = rightRearMotor;
        m_xPidCtrl = xPidCtrl;
        m_yPidCtrl = yPidCtrl;
        m_turnPidCtrl = turnPidCtrl;
        m_gyro = gyro;
        m_motorPosition = motorPosition;

        m_xDistPerRev = 1.0;
        m_yDistPerRev = 1.0;
        m_degPerRev = 360.0;
        m_xPos = 0.0;
        m_yPos = 0.0;
        m_rotPos = 0.0;
        m_heading = 0.0;
        m_driveBaseFlags = 0;

        ResetPosition();

        SetExpiration(1.0);
        SetSafetyEnabled(false);

        UINT32 taskFlags = TASK_START_MODE | TASK_STOP_MODE;
        if ((gyro != NULL) || (motorPosition != NULL))
        {
            taskFlags |= TASK_PRE_PERIODIC;
        }
        RegisterTask(MOD_NAME, taskFlags);

#ifdef _DEBUG_DRIVEBASE
        RegisterCmdHandler(MOD_NAME, NULL, m_varTable);
#endif

#ifdef _LOGDATA_DRIVEBASE
        DataLogger *dataLogger = DataLogger::GetInstance();
        dataLogger->AddDataPoint(MOD_NAME, "", "xPos", "%f",
                                 DataFloat, &m_xPos);
        dataLogger->AddDataPoint(MOD_NAME, "", "yPos", "%f",
                                 DataFloat, &m_yPos);
        dataLogger->AddDataPoint(MOD_NAME, "", "rotPos", "%f",
                                 DataFloat, &m_rotPos);
        dataLogger->AddDataPoint(MOD_NAME, "", "heading", "%f",
                                 DataFloat, &m_heading);
#endif

        TExit();
    }   //CommonInit

public:
#ifdef _DEBUG_DRIVEBASE
    static VAR_ENTRY    m_varTable[];
#endif

    /**
     *  This function resets the robot position.
     */
    void
    ResetPosition(
        void
        )
    {
        TLevel(API);
        TEnter();

        if (m_motorPosition != NULL)
        {
            m_startPos[IDX_LEFT_REAR] =
                m_motorPosition->GetMotorPosition(m_motors[IDX_LEFT_REAR]);
            m_startPos[IDX_RIGHT_REAR] =
                m_motorPosition->GetMotorPosition(m_motors[IDX_RIGHT_REAR]);
            if (m_driveBaseFlags & DRIVEF_FOUR_MOTORS)
            {
                m_startPos[IDX_LEFT_FRONT] =
                    m_motorPosition->GetMotorPosition(
                        m_motors[IDX_LEFT_FRONT]);
                m_startPos[IDX_RIGHT_FRONT] =
                    m_motorPosition->GetMotorPosition(
                        m_motors[IDX_RIGHT_FRONT]);
            }
        }

        if (m_gyro != NULL)
        {
            m_gyro->Reset();
        }

        TExit();
        return;
    }   //ResetPosition

    /**
     *  This function sets the encoder polarities.
     *
     *  @param leftReversed Specifies the left encoder is reversed.
     *  @param rightReversed Specifies the right encoder is reversed.
     */
    void
    SetEncoderPolarities(
        bool leftReversed,
        bool rightReversed
        )
    {
        TLevel(API);
        TEnterMsg(("leftReversed=%d,rightReversed=%d",
                   leftReversed, rightReversed));

        m_encPolarities[IDX_LEFT_REAR] = leftReversed? -1: 1;
        m_encPolarities[IDX_RIGHT_REAR] = rightReversed? -1: 1;

        TExit();
    }   //SetEncoderPolarities

    /**
     *  This function sets the encoder polarities.
     *
     *  @param leftFrontReversed Specifies the left front encoder is reversed.
     *  @param leftRearReversed Specifies the left rear encoder is reversed.
     *  @param rightFrontReversed Specifies the right front encoder is reversed.
     *  @param rightRearReversed Specifies the right rear encoder is reversed.
     */
    void
    SetEncoderPolarities(
        bool leftFrontReversed,
        bool leftRearReversed,
        bool rightFrontReversed,
        bool rightRearReversed
        )
    {
        TLevel(API);
        TEnterMsg(("lfReversed=%d,lrReversed=%d,rfReversed=%d,rrReversed=%d",
                   leftFrontReversed, leftRearReversed,
                   rightFrontReversed, rightRearReversed));

        m_encPolarities[IDX_LEFT_FRONT] = leftFrontReversed? -1: 1;
        m_encPolarities[IDX_LEFT_REAR] = leftRearReversed? -1: 1;
        m_encPolarities[IDX_RIGHT_FRONT] = rightFrontReversed? -1: 1;
        m_encPolarities[IDX_RIGHT_REAR] = rightRearReversed? -1: 1;

        TExit();
    }   //SetEncoderPolarities

    /**
     *  This function sets the distance per encoder revolution scale factor
     *  so that the GetPosition function can return the position values in
     *  correct units. It also sets the degree per encoder revolution scale
     *  factor so that the heading value returned will be in correct degrees.
     *
     *  @param xDistPerRev Specifies the X distance per encoder revolution.
     *  @param yDistPerRev Specifies the Y distance per encoder revolution.
     *  @param degPerRev Specifies robot degrees turned per encoder revolution.
     */
    void
    SetEncoderScale(
        float xDistPerRev,
        float yDistPerRev,
        float degPerRev
        )
    {
        TLevel(API);
        TEnterMsg(("xDistPerRev=%5.2f,yDistPerRev=%5.2f,degPerRev=%5.2f",
                   xDistPerRev, yDistPerRev, degPerRev));

        m_xDistPerRev = xDistPerRev;
        m_yDistPerRev = yDistPerRev;
        m_degPerRev = degPerRev;

        TExit();
    }   //SetEncoderScale

    /**
     *  This function returns the current robot position values.
     *
     *  @param xPos Points to a variable to receive the x position.
     *  @param yPos Points to a variable to receive the y position.
     *  @param rotPos Points to a variable to receive the rotational position.
     *  @param heading Points to a variable to receive the robot heading.
     */
    void
    GetPosition(
        float *xPos,
        float *yPos,
        float *rotPos,
        float *heading
        )
    {
        TLevel(FUNC);
        TEnterMsg(("xPos=%p,yPos=%p,rotPos=%p,heading=%p",
                   xPos, yPos, rotPos, heading));

        if (xPos != NULL)
        {
            *xPos = m_xPos;
        }

        if (yPos != NULL)
        {
            *yPos = m_yPos;
        }

        if (rotPos != NULL)
        {
            *rotPos = m_rotPos;
        }

        if (heading != NULL)
        {
            *heading = m_heading;
        }

        TExitMsg(("!(x=%5.1f,y=%5.1f,rot=%5.1f,heading=%5.1f)",
                  m_xPos, m_yPos, m_rotPos, m_heading));
        return;
    }   //GetPosition

    /**
     *  This function is called from the PID controllers to get the PID input
     *  value.
     *
     *  @param pidCtrl Specifies the PID controller that needs to read its
     *         input sensor.
     *
     *  @return Read and return the sensor value corresponding to the PID
     *          controller.
     */
    float
    GetInput(
        TrcPIDCtrl *pidCtrl
        )
    {
        float input = 0.0;

        TLevel(CALLBK);
        TEnterMsg(("pidCtrl=%p", pidCtrl));

        if (pidCtrl == m_xPidCtrl)
        {
            input = m_xPos;
        }
        else if (pidCtrl == m_yPidCtrl)
        {
            input = m_yPos;
        }
        else if (pidCtrl == m_turnPidCtrl)
        {
            input = (m_gyro != NULL)? m_gyro->GetAngle(): m_rotPos;
        }

        TExitMsg(("=%f", input));
        return input;
    }   //GetInput

    /**
     *  This function stops the drive base.
     */
    void
    Stop(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_pidDrive.Stop();

        TExit();
    }   //Stop

    /**
     *  This function overrides the one in the RobotDrive class. It seems
     *  there is a bug in RobotDrive::ArcadeDrive where positive rotateValue
     *  causes the robot to turn left instead of right. This is opposite to
     *  convention. Therefore, we are correcting this bug by overriding the
     *  function here.
     *
     *  @param moveValue Specifies the value to use for fowards/backwards.
     *  @param rotateValue Specifies the value to use for the rotate right/left.
     *  @param squaredInputs If set, increases the sensitivity at low speeds.
     */
    void
    ArcadeDrive(
        float moveValue,
        float rotateValue,
        bool squaredInputs = true)
    {
        TLevel(API);
        TEnterMsg(("move=%f,rot=%f,squared=%x",
                   moveValue, rotateValue, squaredInputs));

        RobotDrive::ArcadeDrive(moveValue, -rotateValue, squaredInputs);

        TExit();
        return;
    }   //ArcadeDrive

    /**
     *  This function sets PID drive target with the given drive distance and
     *  turn angle setpoints.
     *
     *  @param distXSetPoint Specifies the lateral target distance relative
     *         to current position.
     *  @param distYSetPoint Specifies the forward target distance relative
     *         to current position.
     *  @param angleSetPoint Specifies the target angle relative to current
     *         angle.
     *  @param fHoldTarget Optionally specifies if Drive will maintain target
     *         or will stop when target is reached.
     *  @param notifyEvent Specifies the event to notifying for completion.
     *  @param timeout Specifies the timeout in msec. No timeout if zero.
     */
    void
    DriveSetTarget(
        float  distXSetPoint,
        float  distYSetPoint,
        float  angleSetPoint,
        bool   fHoldTarget = false,
        Event *notifyEvent = NULL,
        UINT32 timeout = 0
        )
    {
        TLevel(API);
        TEnterMsg(("distXSetPt=%f,distYSetPt=%f,angleSetPt=%f,"
                   "fHoldTarget=%x,event=%p,timeout=%d",
                   distXSetPoint, distYSetPoint, angleSetPoint, fHoldTarget,
                   notifyEvent, timeout));

        m_pidDrive.SetTarget(distXSetPoint,
                             distYSetPoint,
                             angleSetPoint,
                             fHoldTarget,
                             notifyEvent,
                             timeout);

        TExit();
        return;
    }   //DriveSetTarget

    /**
     *  This function aborts the PID drive operation in progress and will
     *  cause PID drive to send a drive complete notification.
     */
    void
    AbortPidDrive(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_pidDrive.Abort();

        TExit();
        return;
    }   //AbortPidDrive

    /**
     *  Constructor for two-motor drive.
     *
     *  @param leftMotor Specifies the left wheel motor controller.
     *  @param rightMotor Specifies the right wheel motor controller.
     *  @param yPidCtrl Specifies the PID controller for forward/backward.
     *  @param turnPidCtrl Specifies the PID controller for turning.
     *  @param gyro Specifies the gyro used to keep track of the heading.
     *  @param motorPosition Specifies the MotorPosition provider object.
     */
    DriveBase(
        SpeedController *leftMotor,
        SpeedController *rightMotor,
        TrcPIDCtrl      *yPidCtrl = NULL,
        TrcPIDCtrl      *turnPidCtrl = NULL,
        Gyro            *gyro = NULL,
        MotorPosition   *motorPosition = NULL
        ): RobotDrive(leftMotor, rightMotor)
         , m_pidDrive(this, NULL, yPidCtrl, turnPidCtrl, this)
    {
        TLevel(INIT);
        TEnter();

        CommonInit(NULL, leftMotor, NULL, rightMotor,
                   NULL, yPidCtrl, turnPidCtrl,
                   gyro, motorPosition);

        TExit();
    }   //DriveBase

    /**
     *  Constructor for four-motor drive.
     *
     *  @param leftFrontMotor Specifies the left front motor controller.
     *  @param leftRearMotor Specifies the left rear motor controller.
     *  @param rightFrontMotor Specifies the right front motor controller.
     *  @param rightRearMotor Specifies the right rear motor controller.
     *  @param xPidCtrl Specifies the PID controller for sideway.
     *  @param yPidCtrl Specifies the PID controller for forward/backward.
     *  @param turnPidCtrl Specifies the PID controller for turning.
     *  @param gyro Specifies the gyro used to keep track of the heading.
     *  @param motorPosition Specifies the MotorPosition provider object.
     */
    DriveBase(
        SpeedController *leftFrontMotor,
        SpeedController *leftRearMotor,
        SpeedController *rightFrontMotor,
        SpeedController *rightRearMotor,
        TrcPIDCtrl      *xPidCtrl = NULL,
        TrcPIDCtrl      *yPidCtrl = NULL,
        TrcPIDCtrl      *turnPidCtrl = NULL,
        Gyro            *gyro = NULL,
        MotorPosition   *motorPosition = NULL
        ): RobotDrive(leftFrontMotor, leftRearMotor,
                      rightFrontMotor, rightRearMotor)
         , m_pidDrive(this, xPidCtrl, yPidCtrl, turnPidCtrl, this)
    {
        TLevel(INIT);
        TEnter();

        CommonInit(leftFrontMotor,
                   leftRearMotor,
                   rightFrontMotor,
                   rightRearMotor,
                   xPidCtrl,
                   yPidCtrl,
                   turnPidCtrl,
                   gyro,
                   motorPosition);
        if (xPidCtrl != NULL)
        {
            m_driveBaseFlags |= DRIVEF_MECANUM;
        }
        m_driveBaseFlags |= DRIVEF_FOUR_MOTORS;

        TExit();
    }   //DriveBase

    /**
     *  Destructor.
     */
    ~DriveBase(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        SetSafetyEnabled(false);
        Stop();
#ifdef _DEBUG_DRIVEBASE
        UnregisterCmdHandler();
#endif    
        UnregisterTask(); 

        TExit();
    }   //~DriveBase

    /**
     *  This function is called by the TaskMgr to start the task.
     *
     *  @param mode Specifies the calling mode (autonomous or teleop).
     */
    void
    TaskStartMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        if (mode != MODE_DISABLED)
        {
            ResetPosition();
            SetSafetyEnabled(true);
        }

        TExit();
    }   //TaskStartMode

    /**
     *  This function is called by the TaskMgr to stop the task.
     *
     *  @param mode Specifies the calling mode (autonomous or teleop).
     */
    void
    TaskStopMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        if (mode != MODE_DISABLED)
        {
            SetSafetyEnabled(false);
            Stop();
        }

        TExit();
    }   //TaskStopMode

    /**
     *  This function is called periodically by the TaskMgr before the main
     *  periodic task.
     *
     *  @param mode Specifies the calling mode (autonomous or teleop).
     */
    void
    TaskPrePeriodic(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        if (m_motorPosition != NULL)
        {
            //
            // According to RobotDrive::MecanumDrive_Cartesian in WPILib:
            //
            // LF =  x + y + rot    RF = -x + y - rot
            // LR = -x + y + rot    RR =  x + y - rot
            //
            // (LF + RR) - (RF + LR) = (2x + 2y) - (-2x + 2y)
            // => (LF + RR) - (RF + LR) = 4x
            // => x = ((LF + RR) - (RF + LR))/4
            //
            // LF + RF + LR + RR = 4y
            // => y = (LF + RF + LR + RR)/4
            //
            // (LF + LR) - (RF + RR) = (2y + 2rot) - (2y - 2rot)
            // => (LF + LR) - (RF + RR) = 4rot
            // => rot = ((LF + LR) - (RF + RR))/4
            //
            float lfEnc, rfEnc, lrEnc, rrEnc;

            lrEnc = m_encPolarities[IDX_LEFT_REAR]*
                    (m_motorPosition->GetMotorPosition(
                        m_motors[IDX_LEFT_REAR]) - m_startPos[IDX_LEFT_REAR]);
            rrEnc = m_encPolarities[IDX_RIGHT_REAR]*
                    (m_motorPosition->GetMotorPosition(
                        m_motors[IDX_RIGHT_REAR]) - m_startPos[IDX_RIGHT_REAR]);
            if (m_driveBaseFlags & DRIVEF_FOUR_MOTORS)
            {
                lfEnc = m_encPolarities[IDX_LEFT_FRONT]*
                        (m_motorPosition->GetMotorPosition(
                            m_motors[IDX_LEFT_FRONT]) -
                         m_startPos[IDX_LEFT_FRONT]);
                rfEnc = m_encPolarities[IDX_RIGHT_FRONT]*
                        (m_motorPosition->GetMotorPosition(
                            m_motors[IDX_RIGHT_FRONT]) -
                         m_startPos[IDX_RIGHT_FRONT]);
                m_xPos = ((lfEnc + rrEnc) - (rfEnc + lrEnc))*m_xDistPerRev/4.0;
                m_yPos = (lfEnc + rfEnc + lrEnc + rrEnc)*m_yDistPerRev/4.0;
                m_rotPos = ((lfEnc + lrEnc) - (rfEnc + rrEnc))*m_degPerRev/4.0;
            }
            else
            {
                m_yPos = (lrEnc + rrEnc)*m_xDistPerRev/2.0;
                m_rotPos = (lrEnc - rrEnc)*m_degPerRev/2.0;
            }
        }

        if (m_gyro != NULL)
        {
            m_heading = m_gyro->GetAngle();
        }

#ifdef _DEBUG_DRIVEBASE
        LCDPrintf((LCD_LINE3, "lf=%5.1f,rf=%5.1f", lfEnc, rfEnc));
        LCDPrintf((LCD_LINE4, "lr=%5.1f,rr=%5.1f", lrEnc, rrEnc));
        LCDPrintf((LCD_LINE5, "x=%5.1f,y=%5.1f", m_xPos, m_yPos));
        LCDPrintf((LCD_LINE6, "r=%5.1f,h=%5.1f", m_rotPos, m_heading));
#endif

        TExit();
    }   //TaskPrePeriodic

#ifdef _DEBUG_DRIVEBASE
    /**
     *  This function prints the value of the variable.
     *
     *  @param varEntry Points to the variable table entry.
     *
     *  @return Success Returns ERR_SUCCESS.
     *  @return Failure Returns error code.
     */
    int
    GetVariable(
        PVAR_ENTRY varEntry
        )
    {
        int rc = ERR_SUCCESS;
        double Kp, Ki, Kd, Kf;

        TLevel(CALLBK);
        TEnterMsg(("var=%s", varEntry->varName));

        switch (varEntry->varID)
        {
            case VARID_XDRIVE_PID:
                m_xPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Kp=%10.8f, Ki=%10.8f, Kd=%10.8f, Kf=%10.8f\n",
                       Kp, Ki, Kd, Kf);
                break;

            case VARID_XDRIVE_KP:
                m_xPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Kp=%10.8f\n", Kp);
                break;

            case VARID_XDRIVE_KI:
                m_xPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Ki=%10.8f\n", Ki);
                break;

            case VARID_XDRIVE_KD:
                m_xPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Kd=%10.8f\n", Kd);
                break;

            case VARID_XDRIVE_KF:
                m_xPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Kf=%10.8f\n", Kf);
                break;

            case VARID_YDRIVE_PID:
                m_yPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Kp=%10.8f, Ki=%10.8f, Kd=%10.8f, Kf=%10.8f\n",
                       Kp, Ki, Kd, Kf);
                break;

            case VARID_YDRIVE_KP:
                m_yPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Kp=%10.8f\n", Kp);
                break;

            case VARID_YDRIVE_KI:
                m_yPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Ki=%10.8f\n", Ki);
                break;

            case VARID_YDRIVE_KD:
                m_yPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Kd=%10.8f\n", Kd);
                break;

            case VARID_YDRIVE_KF:
                m_yPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Kf=%10.8f\n", Kf);
                break;

            case VARID_TURN_PID:
                m_turnPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Kp=%10.8f, Ki=%10.8f, Kd=%10.8f, Kf=%10.8f\n",
                       Kp, Ki, Kd, Kf);
                break;

            case VARID_TURN_KP:
                m_turnPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Kp=%10.8f\n", Kp);
                break;

            case VARID_TURN_KI:
                m_turnPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Ki=%10.8f\n", Ki);
                break;

            case VARID_TURN_KD:
                m_turnPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Kd=%10.8f\n", Kd);
                break;

            case VARID_TURN_KF:
                m_turnPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                printf("Kf=%10.8f\n", Kf);
                break;

            default:
                TErr(("Invalid variable ID (var=%s,ID=%d).\n",
                      varEntry->varName, varEntry->varID));
                rc = ERR_ASSERT;
        }

        TExitMsg(("=%d", rc));
        return rc;
    }   //GetVariable

    /**
     *  This function sets the value of the variable.
     *
     *  @param varEntry Points to the variable table entry.
     *  @param varData Points to the data to be set to the variable.
     *
     *  @return Success Returns ERR_SUCCESS.
     *  @return Failure Returns error code.
     */
    int
    SetVariable(
        PVAR_ENTRY varEntry,
        void *varData
        )
    {
        int rc = ERR_SUCCESS;
        double Kp, Ki, Kd, Kf;

        TLevel(CALLBK);
        TEnterMsg(("var=%s,varData=%p", varEntry->varName, varData));

        switch (varEntry->varID)
        {
            case VARID_XDRIVE_KP:
                m_xPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                Kp = *(double *)varData;
                m_xPidCtrl->SetPID(Kp, Ki, Kd, Kf);
                break;

            case VARID_XDRIVE_KI:
                m_xPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                Ki = *(double *)varData;
                m_xPidCtrl->SetPID(Kp, Ki, Kd, Kf);
                break;

            case VARID_XDRIVE_KD:
                m_xPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                Kd = *(double *)varData;
                m_xPidCtrl->SetPID(Kp, Ki, Kd, Kf);
                break;

            case VARID_XDRIVE_KF:
                m_xPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                Kf = *(double *)varData;
                m_xPidCtrl->SetPID(Kp, Ki, Kd, Kf);
                break;

            case VARID_YDRIVE_KP:
                m_yPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                Kp = *(double *)varData;
                m_yPidCtrl->SetPID(Kp, Ki, Kd, Kf);
                break;

            case VARID_YDRIVE_KI:
                m_yPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                Ki = *(double *)varData;
                m_yPidCtrl->SetPID(Kp, Ki, Kd, Kf);
                break;

            case VARID_YDRIVE_KD:
                m_yPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                Kd = *(double *)varData;
                m_yPidCtrl->SetPID(Kp, Ki, Kd, Kf);
                break;

            case VARID_YDRIVE_KF:
                m_yPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                Kf = *(double *)varData;
                m_yPidCtrl->SetPID(Kp, Ki, Kd, Kf);
                break;

            case VARID_TURN_KP:
                m_turnPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                Kp = *(double *)varData;
                m_turnPidCtrl->SetPID(Kp, Ki, Kd, Kf);
                break;

            case VARID_TURN_KI:
                m_turnPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                Ki = *(double *)varData;
                m_turnPidCtrl->SetPID(Kp, Ki, Kd, Kf);
                break;

            case VARID_TURN_KD:
                m_turnPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                Kd = *(double *)varData;
                m_turnPidCtrl->SetPID(Kp, Ki, Kd, Kf);
                break;

            case VARID_TURN_KF:
                m_turnPidCtrl->GetPID(&Kp, &Ki, &Kd, &Kf);
                Kf = *(double *)varData;
                m_turnPidCtrl->SetPID(Kp, Ki, Kd, Kf);
                break;

            default:
                TErr(("Error: invalid variable ID (var=%s,ID=%d).\n",
                      varEntry->varName, varEntry->varID));
                rc = ERR_ASSERT;
        }

        TExitMsg(("=%d", rc));
        return rc;
    }   //SetVariable
#endif

};  //class DriveBase

#ifdef _DEBUG_BASEDRIVE
VAR_ENTRY DriveBase::m_varTable[] =
{
    {"XDrivePID", VARID_XDRIVE_PID, VarDouble, NULL, 0, NULL,
     "Get XDrive PID constants"},
    {"XDriveKp",  VARID_XDRIVE_KP,  VarDouble, NULL, 0, NULL,
     "Get/Set XDrive Kp constant"},
    {"XDriveKi",  VARID_XDRIVE_KI,  VarDouble, NULL, 0, NULL,
     "Get/Set XDrive Ki constant"},
    {"XDriveKd",  VARID_XDRIVE_KD,  VarDouble, NULL, 0, NULL,
     "Get/Set XDrive Kd constant"},
    {"XDriveKf",  VARID_XDRIVE_KF,  VarDouble, NULL, 0, NULL,
     "Get/Set XDrive Kf constant"},
    {"YDrivePID", VARID_YDRIVE_PID, VarDouble, NULL, 0, NULL,
     "Get YDrive PID constants"},
    {"YDriveKp",  VARID_YDRIVE_KP,  VarDouble, NULL, 0, NULL,
     "Get/Set YDrive Kp constant"},
    {"YDriveKi",  VARID_YDRIVE_KI,  VarDouble, NULL, 0, NULL,
     "Get/Set YDrive Ki constant"},
    {"YDriveKd",  VARID_YDRIVE_KD,  VarDouble, NULL, 0, NULL,
     "Get/Set YDrive Kd constant"},
    {"YDriveKf",  VARID_YDRIVE_KF,  VarDouble, NULL, 0, NULL,
     "Get/Set YDrive Kf constant"},
    {"TurnPID", VARID_TURN_PID,     VarDouble, NULL, 0, NULL,
     "Get Turn PID constants"},
    {"TurnKp",  VARID_TURN_KP,      VarDouble, NULL, 0, NULL,
     "Get/Set Turn Kp constant"},
    {"TurnKi",  VARID_TURN_KI,      VarDouble, NULL, 0, NULL,
     "Get/Set Turn Ki constant"},
    {"TurnKd",  VARID_TURN_KD,      VarDouble, NULL, 0, NULL,
     "Get/Set Turn Kd constant"},
    {"TurnKf",  VARID_TURN_KF,      VarDouble, NULL, 0, NULL,
     "Get/Set Turn Kf constant"},
    {NULL,       0,                 VarNone,   NULL, 0, NULL, NULL}
};
#endif

#endif  //ifndef _DRIVEBASE_H
    
