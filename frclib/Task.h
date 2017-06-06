#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Task.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     CoopTask and TaskMgr classes.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TASK_H
#define _TASK_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_TASK
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Task"

//
// Constants.
//
#define MAX_NUM_TASKS           32
#define MAX_TASKNAME_LEN        15

#define TASK_START_MODE         0x00000001
#define TASK_STOP_MODE          0x00000002
#define TASK_PRE_PERIODIC       0x00000004
#define TASK_POST_PERIODIC      0x00000008
#define TASK_PRE_CONTINUOUS     0x00000010
#define TASK_POST_CONTINUOUS    0x00000020

#define MODE_DISABLED           0
#define MODE_AUTONOMOUS         1
#define MODE_TELEOP             2
#define MODE_TEST               3

/**
 * This abstract class defines the CoopTask object. The object is a callback
 * interface. It is not meant to be created as an object. Instead, it should
 * be inherited by a subclass who needs to be called as a cooperative task.
 */
class CoopTask
{
public:
    /**
     * This function registers a CoopTask object.
     *
     * @param taskName Specifies the name of the task.
     * @param flags Specifies the CoopTask callback options.
     *
     * @return Returns true if the CoopTask is successfully registered, false
     *         otherwise.
     */
    bool
    RegisterTask(
        char  *taskName,
        UINT32 flags
        );

    /**
     * This function unregisters a CoopTask object.
     *
     * @return Returns true if the CoopTask is successfully unregistered, false
     *         otherwise.
     */
    bool
    UnregisterTask(
        void
        );

    /**
     * This function is called to start a CoopTask.  Called at the start of
     * Autonomous and TeleOp.
     *
     * @param mode Specifies the caller mode.
     */
    virtual
    void
    TaskStartMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));
        TExit();
        return;
    }   //TaskStartMode

    /**
     * This function is called to stop a CoopTask.  Called at the end of
     * Autonomous and TeleOp.
     *
     * @param mode Specifies the caller mode.
     */
    virtual
    void
    TaskStopMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));
        TExit();
        return;
    }   //TaskStopMode

    /**
     * This function is called periodically before the main periodic task.
     * It typically contains code to process inputs and sensors.
     *
     * @param mode Specifies the caller mode.
     */
    virtual
    void
    TaskPrePeriodic(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));
        TExit();
        return;
    }   //TaskPrePeriodic

    /**
     * This function is called periodically after the main periodic task.
     * It typically contains code to process outputs such as the motors
     * and actuators.
     *
     * @param mode Specifies the caller mode.
     */
    virtual
    void
    TaskPostPeriodic(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));
        TExit();
        return;
    }   //TaskPostPeriodic

    /**
     * This function is called periodically before the main continuous task.
     * It typically contains code to process inputs and sensors that require
     * higher precision than the periodic task.
     *
     * @param mode Specifies the caller mode.
     */
    virtual
    void
    TaskPreContinuous(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));
        TExit();
        return;
    }   //TaskPreContinuous

    /**
     * This function is called periodically after the main continuous task.
     * It typically contains code to process outputs that require higher
     * precision than the periodic task.
     *
     * @param mode Specifies the caller mode.
     */
    virtual
    void
    TaskPostContinuous(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));
        TExit();
        return;
    }   //TaskPostContinuous

};  //class CoopTask

/**
 * This class defines and implements the TaskMgr object.
 */
class TaskMgr
{
private:
    static TaskMgr *m_instance;
    int             m_numTasks;
    char            m_taskNames[MAX_NUM_TASKS][MAX_TASKNAME_LEN + 1];
    CoopTask       *m_tasks[MAX_NUM_TASKS];
    UINT32          m_taskFlags[MAX_NUM_TASKS];

    /**
     * This function finds the task in the registered task list.
     *
     * @param task Specifies the registered CoopTask to lookup.
     *
     * @return If found, the task index is returned, otherwise it returns -1.
     */
    int
    FindTask(
        CoopTask  *task
        )
    {
        int index = -1;

        for (int i = 0; i < m_numTasks; i++)
        {
            if (task == m_tasks[i])
            {
                index = i;
                break;
            }
        }

        return index;
    }   //FindTask

protected:
    /**
     * Constructor: Create an instance of the TaskMgr object.
     */
    TaskMgr(
        void
        ): m_numTasks(0)
    {
        TLevel(INIT);
        TEnter();

        for (int idx = 0; idx < MAX_NUM_TASKS; idx++)
        {
            m_taskNames[idx][0] = '\0';
            m_tasks[idx] = NULL;
            m_taskFlags[idx] = 0;
        }

        TExit();
    }   //TaskMgr

public:
    /**
     * Destructor: Destroy an instance of the TaskMgr object.
     */
    ~TaskMgr(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~TaskMgr

    /**
     * This function returns the global instance of the TaskMgr, create it
     * if necessary.
     */
    static
    TaskMgr *
    GetInstance(
        void
        )
    {
        TLevel(API);
        TEnter();

        if (m_instance == NULL)
        {
            m_instance = new TaskMgr();
        }

        TExitMsg(("=%p", m_instance));
        return m_instance;
    }   //GetInstance

    /**
     * This function deletes the global instance of the TaskMgr if it exists.
     */
    static
    void
    DeleteInstance(
        void
        )
    {
        TLevel(API);
        TEnter();

        SAFE_DELETE(m_instance);

        TExit();
        return;
    }   //DeleteInstance

    /**
     * This function registers a CoopTask object.
     *
     * @param taskName Specifies the name of the task.
     * @param task Specifies the CoopTask to be registered with TaskMgr.
     * @param flags Specifies the CoopTask callback types.
     *
     * @return Returns true if the CoopTask is successfully registered, false
     *         otherwise.
     */
    bool
    RegisterTask(
        char      *taskName,
        CoopTask  *task,
        UINT32    flags
        )
    {
        bool rc = false;
        int index;

        TLevel(API);
        TEnterMsg(("task=%p,flags=%x", task, flags));

        index = FindTask(task);
        if (index != -1)
        {
            //
            // The task is already registered, just add to the mask.
            //
            m_taskFlags[index] |= flags;
            rc = true;
        }
        else if (m_numTasks < MAX_NUM_TASKS)
        {
            strncpy(&m_taskNames[m_numTasks][0], taskName, MAX_TASKNAME_LEN);
            m_taskNames[m_numTasks][MAX_TASKNAME_LEN] = '\0';
            m_tasks[m_numTasks] = task;
            m_taskFlags[m_numTasks] = flags;
            m_numTasks++;
            rc = true;
        }

        TExitMsg(("=%x", rc));
        return rc;
    }   //RegisterTask

    /**
     * This function unregisters a CoopTask object.
     *
     * @param task Specifies the CoopTask to be unregistered with TaskMgr.
     *
     * @return Returns true if the CoopTask is successfully unregistered,
     *         false otherwise.
     */
    bool
    UnregisterTask(
        CoopTask  *task
        )
    {
        bool rc = false;
        int i, j;

        TLevel(API);
        TEnterMsg(("task=%p", task));

        i = FindTask(task);
        if (i != -1)
        {
            //
            // Found the task, remove it from the registered list.
            //
            for (j = i + 1; j < m_numTasks; j++)
            {
                strcpy(&m_taskNames[j - 1][0], &m_taskNames[j][0]);
                m_tasks[j - 1] = m_tasks[j];
                m_taskFlags[j - 1] = m_taskFlags[j];
            }
            m_taskNames[j - 1][0] = '\0';
            m_tasks[j - 1] = NULL;
            m_taskFlags[j - 1] = 0;
            m_numTasks--;
            rc = true;
        }

        TExitMsg(("=%x", rc));
        return rc;
    }   //UnregisterTask

    /**
     * This function calls all the registered start mode tasks.
     *
     * @param mode Specifies the caller mode.
     */
    void
    TaskStartModeAll(
        UINT32 mode
        )
    {
        TLevel(API);
        TEnterMsg(("mode=%d", mode));

        for (int idx = 0; idx < m_numTasks; idx++)
        {
            if (m_taskFlags[idx] & TASK_START_MODE)
            {
                m_tasks[idx]->TaskStartMode(mode);
            }
        }

        TExit();
        return;
    }   //TaskStartModeAll

    /**
     * This function calls all the registered stop mode tasks.
     *
     * @param mode Specifies the caller mode.
     */
    void
    TaskStopModeAll(
        UINT32 mode
        )
    {
        TLevel(API);
        TEnterMsg(("mode=%d", mode));

        for (int idx = m_numTasks - 1; idx >= 0; idx--)
        {
            if (m_taskFlags[idx] & TASK_STOP_MODE)
            {
                m_tasks[idx]->TaskStopMode(mode);
            }
        }

        TExit();
        return;
    }   //TaskStopModeAll

    /**
     * This function calls all the registered pre-periodic tasks.
     *
     * @param mode Specifies the caller mode.
     */
    void
    TaskPrePeriodicAll(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        for (int idx = 0; idx < m_numTasks; idx++)
        {
            if (m_taskFlags[idx] & TASK_PRE_PERIODIC)
            {
                m_tasks[idx]->TaskPrePeriodic(mode);
            }
        }

        TExit();
        return;
    }   //TaskPrePeriodicAll

    /**
     * This function calls all the registered post-periodic tasks.
     *
     * @param mode Specifies the caller mode.
     */
    void
    TaskPostPeriodicAll(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        for (int idx = m_numTasks - 1; idx >= 0; idx--)
        {
            if (m_taskFlags[idx] & TASK_POST_PERIODIC)
            {
                m_tasks[idx]->TaskPostPeriodic(mode);
            }
        }

        TExit();
        return;
    }   //TaskPostPeriodicAll

    /**
     * This function calls all the registered pre-continuous tasks.
     *
     * @param mode Specifies the caller mode.
     */
    void
    TaskPreContinuousAll(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        for (int idx = 0; idx < m_numTasks; idx++)
        {
            if (m_taskFlags[idx] & TASK_PRE_CONTINUOUS)
            {
                m_tasks[idx]->TaskPreContinuous(mode);
            }
        }

        TExit();
        return;
    }   //TaskPreContinuousAll

    /**
     * This function calls all the registered post-continuous tasks.
     *
     * @param mode Specifies the caller mode.
     */
    void
    TaskPostContinuousAll(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        for (int idx = m_numTasks - 1; idx >= 0; idx--)
        {
            if (m_taskFlags[idx] & TASK_POST_CONTINUOUS)
            {
                m_tasks[idx]->TaskPostContinuous(mode);
            }
        }

        TExit();
        return;
    }   //TaskPostContinuousAll

};  //class TaskMgr

TaskMgr *TaskMgr::m_instance = NULL;

/**
 * This function registers a CoopTask object.
 *
 * @param taskName Specifies the name of the task.
 * @param flags Specifies the CoopTask callback types.
 *
 * @return Returns true if the CoopTask is successfully registered, false
 *         otherwise.
 */
bool
CoopTask::RegisterTask(
    char  *taskName,
    UINT32 flags
    )
{
    bool rc = false;
    TaskMgr *taskMgr = TaskMgr::GetInstance();

    TLevel(API);
    TEnterMsg(("flags=%x", flags));

    if (taskMgr != NULL)
    {
        rc = taskMgr->RegisterTask(taskName, this, flags);
    }

    TExitMsg(("=%x", rc));
    return rc;
}   //RegisterTask

/**
 * This function unregisters a CoopTask object.
 *
 * @return Returns true if the CoopTask is successfully unregistered, false
 *         otherwise.
 */
bool
CoopTask::UnregisterTask(
    void
    )
{
    bool rc = false;
    TaskMgr *taskMgr = TaskMgr::GetInstance();

    TLevel(API);
    TEnter();

    if (taskMgr != NULL)
    {
        rc = taskMgr->UnregisterTask(this);
    }

    TExitMsg(("=%x", rc));
    return rc;
}   //UnregisterTask

#endif  //ifndef _TASK_H
