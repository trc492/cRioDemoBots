#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Feeder.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     Feeder class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_FEEDER
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "Feeder"

#ifdef _USE_COMPETITION_ROBOT
  #define FEEDER_REVERSE_POWER  -0.5
  #define FEEDER_FORWARD_POWER  1.00
#else
  #define FEEDER_REVERSE_POWER  -0.6
  #define FEEDER_FORWARD_POWER  0.50
#endif

/**
 * This module defines and implements the Feeder subsytem. The Feeder
 * subsystem consists of a feeder motor and a touch switch.
 */
class Feeder: public CoopTask,
              public DigitalInNotify
{
    private:
        DigitalIn      *m_digitalIn;
        Talon           m_feederMotor;
        StateMachine    m_feederSM;
        Event           m_feederEvent;
        Event		   *m_finished;
        bool            m_blockingFrisbee;

    public:
        Feeder(
            DigitalIn *digitalIn
            ): m_digitalIn(digitalIn)
             , m_feederMotor(1, PWM_FEEDER_MOTOR)
             , m_feederSM()
             , m_feederEvent()
             , m_finished(NULL)
             , m_blockingFrisbee(true)
        {
            TLevel(INIT);
            TEnter();

            m_digitalIn->RegisterNotification(this, DInMask(DIN_FEEDER_SWITCH));
            RegisterTask(MOD_NAME, TASK_STOP_MODE | TASK_PRE_PERIODIC);

            TExit();
        }   //Feeder

        virtual
        ~Feeder(
            void
            )
        {
            TLevel(INIT);
            TEnter();

            UnregisterTask();
            m_digitalIn->UnregisterNotification(this);

            TExit();
        }   //~Feeder

        void Spin()
        {
            TLevel(EVENT);
            TEnter();
            
            m_feederMotor.Set(FEEDER_FORWARD_POWER);
            
            TExit();
        }

        void Stop()
        {
            TLevel(EVENT);
            TEnter();

            m_feederSM.Stop();
            m_feederMotor.Set(0.0);
            m_blockingFrisbee = true;
            
            TExit();
        }

        void Quit()
        {
            TLevel(EVENT);
            TEnter();
            
            Stop();
            
            TExit();
        }

        void
        Start(
        	Event *finished = NULL
            )
        {
            TLevel(EVENT);
            TEnter(); 

            m_finished = finished;
            m_feederSM.Start();
            m_blockingFrisbee = false;

            TExit();
        }

        void Reverse()
        {
            m_feederSM.Stop();
            m_feederMotor.Set(FEEDER_REVERSE_POWER);
            m_blockingFrisbee = false;
        }

        void
        Cram(
            Event *finished = NULL
            )
        {
            TLevel(EVENT);
            TEnter();

            m_finished = finished;
            m_feederSM.Start(SMSTATE_STARTED + 1);
            m_blockingFrisbee = false;
            
            TExit();
        }

        void
        TaskPrePeriodic(
            UINT32 mode
            )
        {
            TLevel(HIFREQ);
            TEnter();
            
            if (mode != MODE_DISABLED && m_feederSM.IsReady())
            {
                UINT32 currState = m_feederSM.GetCurrentState();

                switch(currState)
                {
                case SMSTATE_STARTED:
                    Spin();
                    // wait for the limit switch to click
                    m_feederSM.WaitForSingleEvent(&m_feederEvent,
                                                  currState + 1);
                    break;

                case SMSTATE_STARTED + 1:
                    Spin();
                    // wait for the limit switch to unclick
                    m_feederSM.WaitForSingleEvent(&m_feederEvent,
                                                  currState + 1);
                    break;

                case SMSTATE_STARTED + 2:
                    Stop();
                    if (m_finished != NULL)
                    {
                        m_finished->SetEvent();
                    }
                    break;
                }
            }
            
            if(m_blockingFrisbee)
            {
                if(m_digitalIn->GetChannelState(DIN_FEEDER_SWITCH))
                {
                    m_feederMotor.Set(FEEDER_REVERSE_POWER);
                }
                else
                {
                    m_feederMotor.Set(0.0);
                }
            }
            
            TExit();
        }

        void
        NotifyDIn(
            UINT8  module,
            UINT32 channel,
            bool   fActive
            )
        {
            TLevel(API);
            TEnterMsg(("module=%d,channel=0x%x,fActive=%d",
                       module, channel, fActive));
            
            switch(channel)
            {
                case DIN_FEEDER_SWITCH:
                    m_feederEvent.SetEvent();
                    break;
            }
            
            TExit();
        }

};  //Feeder
