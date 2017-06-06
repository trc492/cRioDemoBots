#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="DataLogger.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     DataLogger class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _DATALOGGER_H
#define _DATALOGGER_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_DATALOGGER
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "DataLogger"

#define DEF_LOGFILE_NAME        "DataLog.csv"

#define DEF_LOG_PERIOD          50
#define DATALOGGERF_STARTED     0x00000001

typedef enum
{
    DataInt8    = 0,
    DataInt16   = 1,
    DataInt32   = 2,
    DataFloat   = 3,
    DataDouble  = 4,
    DataPointer = 5
} LogDataType;

/**
 * This class defines and implements the DataLogger object. The DataLogger
 * object creates a log file in storage that is used to log data points.
 */
class DataLogger
{
private:
    typedef struct _DataPoint
    {
        LIST_ENTRY  list;
        char       *objName;
        char       *instanceName;
        char       *dataName;
        char       *formatString;
        LogDataType dataType;
        void       *data;
    } DATAPOINT, *PDATAPOINT;

    static DataLogger  *m_instance;
    FILE               *m_hLogFile;
    UINT32              m_dataLoggerFlags;
    UINT32              m_logPeriod;
    UINT32              m_expiredTime;
    LIST_ENTRY          m_dataList;

protected:
    /**
     * Constructor: Create an instance of the DataLogger object.
     *
     * @param logPeriod Specifies the logging period in msec.
     */
    DataLogger(
        UINT32 logPeriod
        ): m_hLogFile(NULL)
         , m_dataLoggerFlags(0)
         , m_logPeriod(logPeriod)
    {
        TLevel(INIT);
        TEnterMsg(("logPeriod=%d", logPeriod));

        m_expiredTime = GetMsecTime();
        InitializeListHead(&m_dataList);

        TExit();
    }   //DataLogger

public:
    /**
     * Destructor: Destroy an instance of the DataLogger object.
     */
    virtual
    ~DataLogger(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        if (m_hLogFile != NULL)
        {
            fclose(m_hLogFile);
            m_hLogFile = NULL;
        }

        while (!IsListEmpty(&m_dataList))
        {
            PLIST_ENTRY entry = RemoveHeadList(&m_dataList);
            DATAPOINT *dataPoint = CONTAINING_RECORD(entry, DATAPOINT, list);
            SAFE_DELETE(dataPoint->objName);
            SAFE_DELETE(dataPoint->instanceName);
            SAFE_DELETE(dataPoint->dataName);
            SAFE_DELETE(dataPoint->formatString);
            dataPoint->data = NULL;
            delete dataPoint;
        }

        TExit();
    }   //~DataLogger

    /**
     * This function returns the global instance of the DataLogger, create it
     * if necessary.
     *
     * @param logPeriod Specifies the log period, if absent, DEF_LOG_PERIOD
     *        is used.
     */
    static
    DataLogger *
    GetInstance(
        UINT32 logPeriod = DEF_LOG_PERIOD
        )
    {
        TLevel(API);
        TEnter();

        if (m_instance == NULL)
        {
            m_instance = new DataLogger(logPeriod);
        }

        TExitMsg(("=%p", m_instance));
        return m_instance;
    }   //GetInstance

    /**
     * This function deletes the global instance of the DataLogger if
     * it exists.
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
     * This function sets the logging period.
     *
     * @param logPeriod Specifies the new logging period.
     */
    void
    SetPeriod(
        UINT32 logPeriod
        )
    {
        TLevel(API);
        TEnterMsg(("logPeriod=%d", logPeriod));

        m_logPeriod = logPeriod;

        TExit();
        return;
    }   //SetPeriod

    /**
     * This function adds a data point to be monitored by the log.
     *
     * @param objName Specifies the object name.
     * @param instanceName Specifies the object instance name.
     * @param dataName Specifies the data variable name.
     * @param formatString Specifies the format string for the data.
     * @param dataType Specifies the data variable type.
     * @param data Points to the data variable to be logged.
     */
    bool
    AddDataPoint(
        char *objName,
        char *instanceName,
        char *dataName,
        char *formatString,
        LogDataType dataType,
        void *data
        )
    {
        bool rc = false;
        PDATAPOINT dataPoint = NULL;

        TLevel(API);
        TEnterMsg(("objName=%s,instanceName=%s,objName=%s,format=%s,type=%d,"
                   "pData=%p",
                   objName, instanceName, dataName, formatString, dataType,
                   data));

        if ((data == NULL) || (objName == NULL) || (instanceName == NULL) ||
            (dataName == NULL) || (formatString == NULL))
        {
            TErr(("Pointers cannot be NULL."));
        }
        else if (m_dataLoggerFlags & DATALOGGERF_STARTED)
        {
            TErr(("Cannot add data point once the logger has started."));
        }
        else if ((dataPoint = new DATAPOINT) == NULL)
        {
            TErr(("Failed to create data point."));
        }
        else
        {
            memset(dataPoint, 0, sizeof(*dataPoint));
            if ((dataPoint->objName = new char[strlen(objName) + 1]) == NULL)
            {
                TErr(("Failed to create object name string (len=%d).",
                      strlen(objName)));
            }
            else if ((dataPoint->instanceName =
                      new char[strlen(instanceName) + 1]) == NULL)
            {
                TErr(("Failed to create object instance name string (len=%d).",
                      strlen(instanceName)));
            }
            else if ((dataPoint->dataName = new char[strlen(dataName) + 1]) ==
                     NULL)
            {
                TErr(("Failed to create data name string (len=%d).",
                      strlen(dataName)));
            }
            else if ((dataPoint->formatString =
                      new char[strlen(formatString) + 1]) == NULL)
            {
                TErr(("Failed to create format string (len=%d).",
                      strlen(formatString)));
            }
            else
            {
                strcpy(dataPoint->objName, objName);
                strcpy(dataPoint->instanceName, instanceName);
                strcpy(dataPoint->dataName, dataName);
                strcpy(dataPoint->formatString, formatString);
                dataPoint->dataType = dataType;
                dataPoint->data = data;
                InsertTailList(&m_dataList, &dataPoint->list);
                rc = true;
            }

            if (rc == false)
            {
                //
                // Something failed, let's clean up.
                //
                SAFE_DELETE(dataPoint->objName);
                SAFE_DELETE(dataPoint->instanceName);
                SAFE_DELETE(dataPoint->dataName);
                SAFE_DELETE(dataPoint->formatString);
                SAFE_DELETE(dataPoint);
            }
        }

        TExitMsg(("=%x", rc));
        return rc;
    }   //AddDataPoint

    /**
     * This function starts the data logger.
     *
     * @param logFileName Specifies the log file name string. If not provided,
     *        the default log file name is used.
     * 
     * @return Success: Returns true.
     * @return Failure: Returns false.
     */
    bool
    Start(
        char *logFileName = NULL
        )
    {
        bool rc = false;

        TLevel(API);
        TEnterMsg(("file=%s", logFileName? logFileName: "null"));

        if (m_dataLoggerFlags & DATALOGGERF_STARTED)
        {
            //
            // Data logger already started.
            //
            rc = true;
        }
        else
        {
            if (logFileName == NULL)
            {
                logFileName = DEF_LOGFILE_NAME;
            }

            m_hLogFile = fopen(logFileName, "w");
            if (m_hLogFile == NULL)
            {
                TErr(("Failed to open log file <%s>.", logFileName));
            }
            else
            {
                PLIST_ENTRY entry;
                PDATAPOINT dataPoint;

                fprintf(m_hLogFile, "Timestamp");
                for (entry = m_dataList.Flink;
                     entry != &m_dataList;
                     entry = entry->Flink)
                {
                    dataPoint = CONTAINING_RECORD(entry, DATAPOINT, list);
                    fprintf(m_hLogFile, ",%s%s.%s",
                            dataPoint->objName, dataPoint->instanceName,
                            dataPoint->dataName);
                }
                fprintf(m_hLogFile, "\n");
                m_dataLoggerFlags |= DATALOGGERF_STARTED;
                rc = true;
            }
        }

        TExitMsg(("=%x", rc));
        return rc;
    }   //Start

    /**
     * This function stops the data logger.
     *
     * @return Success: Returns true.
     * @return Failure: Returns false.
     */
    bool
    Stop(
        void
        )
    {
        bool rc = false;

        TLevel(API);
        TEnter();

        if (!(m_dataLoggerFlags & DATALOGGERF_STARTED))
        {
            //
            // Data Logger was not started.
            //
            rc = true;
        }
        else if (m_hLogFile == NULL)
        {
            TErr(("No logger file opened!"));
        }
        else
        {
            fclose(m_hLogFile);
            m_hLogFile = NULL;
            m_dataLoggerFlags &= ~DATALOGGERF_STARTED;
            rc = true;
        }

        TExitMsg(("=%x", rc));
        return rc;
    }   //Stop

    /**
     * This function is called by TaskMgr periodically to log the registered
     * data points.
     */
    void
    LoggerTask(
        void
        )
    {
        TLevel(TASK);
        TEnter();

        if ((m_hLogFile != NULL) && (m_dataLoggerFlags & DATALOGGERF_STARTED))
        {
            UINT32 currTime = GetMsecTime();

            if (currTime >= m_expiredTime)
            {
                PLIST_ENTRY entry;
                PDATAPOINT dataPoint;

                m_expiredTime = currTime + m_logPeriod;
                fprintf(m_hLogFile, "%d", currTime);
                for (entry = m_dataList.Flink;
                     entry != &m_dataList;
                     entry = entry->Flink)
                {
                    dataPoint = CONTAINING_RECORD(entry, DATAPOINT, list);
                    fprintf(m_hLogFile, ",");
                    switch (dataPoint->dataType)
                    {
                    case DataInt8:
                        fprintf(m_hLogFile, dataPoint->formatString,
                                *(UINT8 *)dataPoint->data);
                        break;

                    case DataInt16:
                        fprintf(m_hLogFile, dataPoint->formatString,
                                *(UINT16 *)dataPoint->data);
                        break;

                    case DataInt32:
                        fprintf(m_hLogFile, dataPoint->formatString,
                                *(UINT32 *)dataPoint->data);
                        break;

                    case DataFloat:
                        fprintf(m_hLogFile, dataPoint->formatString,
                                *(float *)dataPoint->data);
                        break;

                    case DataDouble:
                        fprintf(m_hLogFile, dataPoint->formatString,
                                *(double *)dataPoint->data);
                        break;

                    case DataPointer:
                        fprintf(m_hLogFile, dataPoint->formatString,
                                dataPoint->data);
                        break;
                    }
                }
                fprintf(m_hLogFile, "\n");
#if 0
                //
                // Ideally, we should flush I/O but it may take too long.
                //
                fflush(m_hLogFile);
#endif
            }
        }

        TExit();
        return;
    }   //LoggerTask

};  //class DataLogger

DataLogger *DataLogger::m_instance = NULL;

#endif  //ifndef _DATALOGGER_H
