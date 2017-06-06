#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Console.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     Console related objects.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _CONSOLE_H
#define _CONSOLE_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_CONSOLE
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Console"

#define MAX_NUM_ARGS            8

#define ERR_SUCCESS             0
#define ERR_EXIT                1
#define ERR_ASSERT              -1
#define ERR_OBJ_NOT_FOUND       -2
#define ERR_CMD_NOT_FOUND       -3
#define ERR_VAR_NOT_FOUND       -4
#define ERR_INVALID_PARAM       -5
#define ERR_NOT_IMPLEMENTED     -6

#define CMDACTION_NONE          0
#define BASECMD_QUIT            (CMDACTION_NONE + 1)
#define BASECMD_HELP            (CMDACTION_NONE + 2)
#define BASECMD_LISTOBJ         (CMDACTION_NONE + 3)
#define BASECMD_LISTVAR         (CMDACTION_NONE + 4)
#define BASECMD_GET             (CMDACTION_NONE + 5)
#define BASECMD_SET             (CMDACTION_NONE + 6)

#define VARID_DATAPTR           0

#ifdef _USE_COLORFONT
  #define ConPrintf(p)          {                           \
                                    printf(ESC_FGB_CYAN);   \
                                    printf p;               \
                                    printf(ESC_NORMAL);     \
                                }
#else
  #define ConPrintf(p)          printf p
#endif

typedef struct _CmdEntry
{
    const char *cmdName;
    int         cmdAction;
    const char *cmdHelp;
} CMD_ENTRY, *PCMD_ENTRY;

typedef enum
{
    VarNone     = 0,
    VarInt8     = 1,
    VarInt16    = 2,
    VarInt32    = 3,
    VarFloat    = 4,
    VarDouble   = 5,
    VarChar     = 6,
    VarString   = 7
} VarType;

typedef struct _VarEntry
{
    char       *varName;
    int         varID;
    VarType     varType;
    void       *pVarData;
    int         dataLen;
    const char *format;
    const char *varHelp;
} VAR_ENTRY, *PVAR_ENTRY;

/**
 * This abstract class defines the CmdHandler object. The object is a callback
 * interface. It is not meant to be created as an object. Instead, it should
 * be inherited by a subclass who needs to be called to process console
 * commands.
 */
class CmdHandler
{
public:
    /**
     * This function registers a console command handler.
     *
     * @param objName Specifies the name identifying the command handler.
     * @param cmdTable Points to the command table to be registered for the
     *        handler.
     * @param varTable Points to the variable table to be registered for the
     *        handler.
     *
     * @return Returns true if the handler is successfully registered, false
     *         otherwise.
     */
    bool
    RegisterCmdHandler(
        char      *objName,
        PCMD_ENTRY cmdTable,
        PVAR_ENTRY varTable
        );

    /**
     * This function unregisters a console command handler.
     *
     * @return Returns true if the handler is successfully unregistered, false
     *         otherwise.
     */
    bool
    UnregisterCmdHandler(
        void
        );

    /**
     * This function executes the console command.
     *
     * @param cmdEntry Points to the command table entry.
     * @param apszArgs Points to the array of command arguments.
     * @param cArgs Specifies the number of command arguments.
     *
     * @return Success Returns ERR_SUCCESS.
     * @return Failure Returns error code.
     */
    virtual
    int
    ExecuteCommand(
        PCMD_ENTRY  cmdEntry,
        char      **apszArgs,
        int         cArgs
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("cmd=%s,pArgs=%p,cArgs=%d",
                   cmdEntry->cmdName, apszArgs, cArgs));
        TExit();
        return ERR_NOT_IMPLEMENTED;
    }   //ExecuteCommand

    /**
     * This function prints the value of a variable.
     *
     * @param varEntry Points to the variable table entry.
     *
     * @return Success Returns ERR_SUCCESS.
     * @return Failure Returns error code.
     */
    virtual
    int
    GetVariable(
        PVAR_ENTRY varEntry
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("var=%s", varEntry->varName));
        TExit();
        return ERR_NOT_IMPLEMENTED;
    }   //GetVariable

    /**
     * This function sets the value of a variable.
     *
     * @param varEntry Points to the variable table entry.
     * @param varData Points to the data to be set to the variable.
     *
     * @return Success Returns ERR_SUCCESS.
     * @return Failure Returns error code.
     */
    virtual
    int
    SetVariable(
        PVAR_ENTRY varEntry,
        void *varData
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("var=%s,pData=%p", varEntry->varName, varData));
        TExit();
        return ERR_NOT_IMPLEMENTED;
    }   //SetVariable

};  //CmdHandler

/**
 * This class defines and implements the ConsoleCommand object. The
 * ConsoleCommand object processes and dispatches console commands to
 * various objects that register as console command objects. This class
 * also inherits the CmdHandler class to provide console base command
 * handling.
 */
class ConsoleCommand: public CmdHandler
{
private:
    typedef struct _CmdObj
    {
        LIST_ENTRY  list;
        CmdHandler *cmdHandler;
        char       *objName;
        PCMD_ENTRY  cmdTable;
        PVAR_ENTRY  varTable;
    } CMD_OBJ, *PCMD_OBJ;

    static ConsoleCommand  *m_instance;
    static CMD_ENTRY        m_baseCmdTable[];
    static VAR_ENTRY        m_baseVarTable[];
    LIST_ENTRY              m_cmdObjList;

    /**
     * This function finds the command object in the object list registered
     * by the command handler.
     *
     * @param cmdHandler Specifies the command handler to look for.
     *
     * @return Success: Returns a pointer to the object.
     * @return Failure: Returns NULL.
     */
    PCMD_OBJ
    FindCmdObjByHandler(
        CmdHandler *cmdHandler
        )
    {
        PCMD_OBJ cmdObj = NULL;
        PLIST_ENTRY entry;

        TLevel(FUNC);
        TEnterMsg(("handler=%p", cmdHandler));

        for (entry = m_cmdObjList.Flink;
             entry != &m_cmdObjList;
             entry = entry->Flink)
        {
            cmdObj = CONTAINING_RECORD(entry, CMD_OBJ, list);
            if (cmdHandler == cmdObj->cmdHandler)
            {
                break;
            }
            else
            {
                cmdObj = NULL;
            }
        }

        TExitMsg(("=%p", cmdObj));
        return cmdObj;
    }   //FindCmdObjByHandler

    /**
     * This function finds the command object in the object list by name.
     *
     * @param objName Specifies the object name of the handler.
     *
     * @return Success: Returns a pointer to the object.
     * @return Failure: Returns NULL.
     */
    PCMD_OBJ
    FindCmdObjByName(
        char *objName
        )
    {
        PCMD_OBJ cmdObj = NULL;
        PLIST_ENTRY entry;

        TLevel(FUNC);
        TEnterMsg(("obj=%s", objName));

        for (entry = m_cmdObjList.Flink;
             entry != &m_cmdObjList;
             entry = entry->Flink)
        {
            cmdObj = CONTAINING_RECORD(entry, CMD_OBJ, list);
            if ((objName != NULL) && (cmdObj->objName != NULL) &&
                (strcmp(objName, cmdObj->objName) == 0) ||
                (objName == NULL) && (cmdObj->objName == NULL))
            {
                break;
            }
            else
            {
                cmdObj = NULL;
            }
        }

        TExitMsg(("=%p", cmdObj));
        return cmdObj;
    }   //FindCmdObjByName

    /**
     * This function finds the command entry in the command table.
     *
     * @param cmdName Points to the command string.
     * @param cmdTable Points to the command table.
     *
     * @return Success: Returns a pointer to the command entry in the table.
     * @return Failure: Returns NULL.
     */
    PCMD_ENTRY
    FindCommand(
        char      *cmdName,
        PCMD_ENTRY cmdTable
        )
    {
        PCMD_ENTRY cmdEntry = NULL;

        TLevel(FUNC);
        TEnterMsg(("cmdName=%s,cmdTable=%p", cmdName, cmdTable));

        if (cmdTable != NULL)
        {
            for (int i = 0; cmdTable[i].cmdName != NULL; i++)
            {
                if (strcmp(cmdName, cmdTable[i].cmdName) == 0)
                {
                    cmdEntry = &cmdTable[i];
                    break;
                }
            }
        }

        TExitMsg(("=%p", cmdEntry));
        return cmdEntry;
    }   //FindCommand

    /**
     * This function finds the variable entry in the variable table.
     *
     * @param name Points to the obj.var name string.
     *
     * @return Success: Returns a pointer to the variable entry in the table.
     * @return Failure: Returns NULL.
     */
    PVAR_ENTRY
    FindVariable(
        char *name
        )
    {
        PVAR_ENTRY varEntry = NULL;
        char *objName;
        char *varName;
        PCMD_OBJ cmdObj;

        TLevel(FUNC);
        TEnterMsg(("name=%s", name));

        varName = strchr(name, '.');
        if (varName == NULL)
        {
            objName = NULL;
            varName = name;
        }
        else
        {
            objName = name;
            *varName = '\0';
            varName++;
        }

        cmdObj = FindCmdObjByName(objName);
        if ((cmdObj != NULL) && (cmdObj->varTable != NULL))
        {
            for (int i = 0; cmdObj->varTable[i].varName != NULL; i++)
            {
                if (strcmp(varName, cmdObj->varTable[i].varName) == 0)
                {
                    varEntry = &cmdObj->varTable[i];
                    break;
                }
            }
        }

        if (varName != name)
        {
            varName--;
            *varName = '.';
        }

        TExitMsg(("=%p", varEntry));
        return varEntry;
    }   //FindVariable

    /**
     * This function executes the console base command.
     *
     * @param cmdEntry Points to the command table entry.
     * @param apszArgs Points to the array of command arguments.
     * @param cArgs Specifies the number of command arguments.
     *
     * @return Success Returns ERR_SUCCESS.
     * @return Failure Returns error code.
     */
    int
    ExecuteCommand(
        PCMD_ENTRY  cmdEntry,
        char      **apszArgs,
        int         cArgs
        )
    {
        int rc = ERR_SUCCESS;

        TLevel(CALLBK);
        TEnterMsg(("cmd=%s,pArgs=%p,cArgs=%d",
                   cmdEntry->cmdName, apszArgs, cArgs));

        switch (cmdEntry->cmdAction)
        {
            case BASECMD_QUIT:
                rc = ERR_EXIT;
                break;

            case BASECMD_HELP:
                if (cArgs > 1)
                {
                    ConPrintf(("Error: invalid options.\n"));
                    rc = ERR_INVALID_PARAM;
                }
                else
                {
                    PCMD_OBJ cmdObj;

                    cmdObj = FindCmdObjByName((cArgs == 0)? NULL: apszArgs[0]);
                    if (cmdObj == NULL)
                    {
                        ConPrintf(("Error: %s: object not found.\n",
                                   apszArgs[0]));
                        rc = ERR_OBJ_NOT_FOUND;
                    }
                    else if (cmdObj->cmdTable != NULL)
                    {
                        for (int i = 0;
                             cmdObj->cmdTable[i].cmdName != NULL;
                             i++)
                        {
                            ConPrintf(("%8s - %s\n",
                                       cmdObj->cmdTable[i].cmdName,
                                       cmdObj->cmdTable[i].cmdHelp));
                        }
                    }
                }
                break;

            case BASECMD_LISTOBJ:
                if (cArgs != 0)
                {
                    ConPrintf(("Error: invalid options.\n"));
                    rc = ERR_INVALID_PARAM;
                }
                else
                {
                    PLIST_ENTRY entry;
                    PCMD_OBJ cmdObj;

                    ConPrintf(("Command Handlers:\n"));
                    for (entry = m_cmdObjList.Flink;
                         entry != &m_cmdObjList;
                         entry = entry->Flink)
                    {
                        cmdObj = CONTAINING_RECORD(entry, CMD_OBJ, list);
                        ConPrintf(("\t%s\n",
                                   cmdObj->objName? cmdObj->objName: "(Base)"));
                    }
                }
                break;

            case BASECMD_LISTVAR:
                if (cArgs > 1)
                {
                    ConPrintf(("Error: invalid options.\n"));
                    rc = ERR_INVALID_PARAM;
                }
                else
                {
                    PCMD_OBJ cmdObj;

                    cmdObj = FindCmdObjByName((cArgs == 0)? NULL: apszArgs[0]);
                    if (cmdObj == NULL)
                    {
                        ConPrintf(("Error: %s: object not found.\n",
                                   apszArgs[0]));
                        rc = ERR_OBJ_NOT_FOUND;
                    }
                    else if (cmdObj->varTable != NULL)
                    {
                        for (int i = 0;
                             cmdObj->varTable[i].varName != NULL;
                             i++)
                        {
                            ConPrintf(("%8s - %s\n",
                                       cmdObj->varTable[i].varName,
                                       cmdObj->varTable[i].varHelp));
                        }
                    }
                }
                break;

            case BASECMD_GET:
                if (cArgs != 1)
                {
                    ConPrintf(("Error: invalid options.\n"));
                    rc = ERR_INVALID_PARAM;
                }
                else
                {
                    PVAR_ENTRY varEntry = FindVariable(apszArgs[0]);

                    if (varEntry == NULL)
                    {
                        ConPrintf(("Error: %s: variable not found.\n",
                                   apszArgs[0]));
                        rc = ERR_VAR_NOT_FOUND;
                    }
                    else if (varEntry->varID == VARID_DATAPTR)
                    {
                        ConPrintf(("%s = ", apszArgs[0]));
                        switch (varEntry->varType)
                        {
                            case VarChar:
                            case VarInt8:
                                ConPrintf((varEntry->format,
                                           *((UINT8*)varEntry->pVarData)));
                                break;

                            case VarInt16:
                                ConPrintf((varEntry->format,
                                           *((UINT16*)varEntry->pVarData)));
                                break;

                            case VarInt32:
                                ConPrintf((varEntry->format,
                                           *((UINT32*)varEntry->pVarData)));
                                break;

                            case VarFloat:
                                ConPrintf((varEntry->format,
                                           *((float*)varEntry->pVarData)));
                                break;

                            case VarDouble:
                                ConPrintf((varEntry->format,
                                           *((double*)varEntry->pVarData)));
                                break;

                            case VarString:
                                ConPrintf((varEntry->format,
                                           (char*)varEntry->pVarData));
                                break;

                            default:
                                ConPrintf(("Error: Invalid variable type (type=%d).\n",
                                           varEntry->varType));
                                rc = ERR_ASSERT;
                                break;
                        }
                    }
                    else
                    {
                        rc = ((CmdHandler *)varEntry->pVarData)->GetVariable(
                                                varEntry);
                    }
                }
                break;

            case BASECMD_SET:
                if (cArgs != 2)
                {
                    ConPrintf(("Error: invalid options.\n"));
                    rc = ERR_INVALID_PARAM;
                }
                else
                {
                    PVAR_ENTRY varEntry = FindVariable(apszArgs[0]);

                    if (varEntry == NULL)
                    {
                        ConPrintf(("Error: %s: variable not found.\n",
                                   apszArgs[0]));
                        rc = ERR_VAR_NOT_FOUND;
                    }
                    else
                    {
                        switch (varEntry->varType)
                        {
                            case VarInt8:
                            case VarInt16:
                            case VarInt32:
                            {
                                char *psz;
                                long data = strtol(apszArgs[1], &psz, 0);

                                if (*psz != '\0')
                                {
                                    ConPrintf(("Error: %s: invalid number.",
                                               apszArgs[1]));
                                    rc = ERR_INVALID_PARAM;
                                }
                                else if (varEntry->varID != VARID_DATAPTR)
                                {
                                    rc = ((CmdHandler *)varEntry->pVarData)->
                                                SetVariable(varEntry, &data);
                                }
                                else if (varEntry->varType == VarInt8)
                                {
                                    *((UINT8*)varEntry->pVarData) =
                                        (UINT8)data;
                                }
                                else if (varEntry->varType == VarInt16)
                                {
                                    *((UINT16*)varEntry->pVarData) =
                                        (UINT16)data;
                                }
                                else
                                {
                                    *((UINT32*)varEntry->pVarData) =
                                        (UINT32)data;
                                }
                                break;
                            }
                                
                            case VarFloat:
                            case VarDouble:
                            {
                                char *psz;
                                double data = strtod(apszArgs[1], &psz);

                                if (*psz != '\0')
                                {
                                    ConPrintf(("Error: %s: invalid number.",
                                               apszArgs[1]));
                                    rc = ERR_INVALID_PARAM;
                                }
                                else if (varEntry->varID != VARID_DATAPTR)
                                {
                                    rc = ((CmdHandler *)varEntry->pVarData)->
                                                SetVariable(varEntry, &data);
                                }
                                else if (varEntry->varType == VarFloat)
                                {
                                    *((float*)varEntry->pVarData) =
                                        (float)data;
                                }
                                else
                                {
                                    *((double*)varEntry->pVarData) = data;
                                }
                                break;
                            }

                            case VarChar:
                                if (strlen(apszArgs[1]) != 1)
                                {
                                    ConPrintf(("Error: %s: invalid character.",
                                               apszArgs[1]));
                                    rc = ERR_INVALID_PARAM;
                                }
                                else if (varEntry->varID != VARID_DATAPTR)
                                {
                                    rc = ((CmdHandler *)varEntry->pVarData)->
                                                SetVariable(varEntry,
                                                            apszArgs[1]);
                                }
                                else
                                {
                                    *((char*)varEntry->pVarData) =
                                        apszArgs[1][0];
                                }
                                break;

                            case VarString:
                                if (strlen(apszArgs[1]) >= 
                                    (unsigned int)varEntry->dataLen)
                                {
                                    ConPrintf(("Error: %s: string too long (expect=%d).",
                                               apszArgs[1], varEntry->dataLen));
                                    rc = ERR_INVALID_PARAM;
                                }
                                else if (varEntry->varID != VARID_DATAPTR)
                                {
                                    rc = ((CmdHandler *)varEntry->pVarData)->
                                                SetVariable(varEntry,
                                                            apszArgs[1]);
                                }
                                else
                                {
                                    strcpy((char*)varEntry->pVarData,
                                           apszArgs[1]);
                                }
                                break;

                            default:
                                ConPrintf(("Error: Invalid variable type (type=%d).\n",
                                           varEntry->varType));
                                rc = ERR_ASSERT;
                                break;
                        }
                    }
                }
                break;
        }

        TExitMsg(("=%d", rc));
        return rc;
    }   //ExecuteCommand

    /**
     * This function processes the built-in command.
     *
     * @param objName Points to the handler object name string.
     * @param cmdName Points to the command string.
     * @param apszArgs Points to the array of command arguments.
     * @param cArgs Specifies the number of arguments in the array.
     *
     * @return Success Returns ERR_SUCCESS.
     * @return Failure Returns error code.
     */
    int
    ProcessCommand(
        char *objName,
        char *cmdName,
        char **apszArgs,
        int cArgs
        )
    {
        int rc = ERR_SUCCESS;
        PCMD_OBJ cmdObj = FindCmdObjByName(objName);

        TLevel(FUNC);
        TEnterMsg(("objName=%s,cmdName=%s,pArgs=%p,cArgs=%d",
                   objName, cmdName, apszArgs, cArgs));

        if (cmdObj == NULL)
        {
            ConPrintf(("Error: %s: object not found.\n", objName));
            rc = ERR_OBJ_NOT_FOUND;
        }
        else
        {
            PCMD_ENTRY cmdEntry = FindCommand(cmdName, cmdObj->cmdTable);

            if (cmdEntry == NULL)
            {
                ConPrintf(("Error: %s: command not found.\n", cmdName));
                ConPrintf(("Usage: help [<object>]\n"));
                rc = ERR_CMD_NOT_FOUND;
            }
            else if (cmdEntry->cmdAction != CMDACTION_NONE)
            {
                rc = cmdObj->cmdHandler->ExecuteCommand(cmdEntry,
                                                        apszArgs,
                                                        cArgs);
            }
        }

        TExitMsg(("=%d", rc));
        return rc;
    }   //ProcessCommand

protected:
    /**
     * Constructor for the class object.
     */
    ConsoleCommand(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        InitializeListHead(&m_cmdObjList);
        RegisterCmdHandler(this, NULL, m_baseCmdTable, m_baseVarTable);

        TExit();
    }   //ConsoleCommand

public:
    /**
     * Destructor for the class object.
     */
    virtual
    ~ConsoleCommand(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        while (!IsListEmpty(&m_cmdObjList))
        {
            PLIST_ENTRY entry = RemoveHeadList(&m_cmdObjList);
            PCMD_OBJ cmdObj = CONTAINING_RECORD(entry, CMD_OBJ, list);

            cmdObj->cmdHandler = NULL;
            SAFE_DELETE(cmdObj->objName);
            cmdObj->cmdTable = NULL;
            cmdObj->varTable = NULL;
            delete cmdObj;
        }

        TExit();
    }   //~ConsoleCommand

    /**
     * This function returns the global instance of ConsoleCommand, creates
     * it if necessary.
     */
    static
    ConsoleCommand *
    GetInstance(
        void
        )
    {
        TLevel(API);
        TEnter();

        if (m_instance == NULL)
        {
            m_instance = new ConsoleCommand();
        }

        TExitMsg(("=%p", m_instance));
        return m_instance;
    }   //GetInstance

    /**
     * This function deletes the global instance of ConsoleCommand if it
     * exists.
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
     * This function registers a console command handler.
     *
     * @param cmdHandler Specifies the handler that will handle the command.
     * @param objName Specifies the name identifying the command handler.
     * @param cmdTable Points to the command table to be registered for
     *        the object.
     * @param varTable Points to the variable table to be registered for
     *        the object.
     *
     * @return Returns true if the handler is successfully registered,
     *         false otherwise.
     */
    bool
    RegisterCmdHandler(
        CmdHandler *cmdHandler,
        char       *objName,
        PCMD_ENTRY  cmdTable,
        PVAR_ENTRY  varTable
        )
    {
        bool rc = true;
        PCMD_OBJ cmdObj;

        TLevel(API);
        TEnterMsg(("handler=%p,objName=%s,cmdTable=%p,varTale=%p",
                   cmdHandler, objName, cmdTable, varTable));

        cmdObj = FindCmdObjByHandler(cmdHandler);
        if (cmdObj != NULL)
        {
            TErr(("Object %s already registered a command handler.", objName));
            rc = false;
        }
        else if ((cmdTable == NULL) && (varTable == NULL))
        {
            TErr(("Must have at least one table."));
            rc = false;
        }
        else if ((cmdObj = new CMD_OBJ) == NULL)
        {
            TErr(("Failed to create a command handler for %s.", objName));
            rc = false;
        }
        else
        {
            memset(cmdObj, 0, sizeof(*cmdObj));
            if (objName != NULL)
            {
                cmdObj->objName = new char[strlen(objName) + 1];
                if (cmdObj->objName == NULL)
                {
                    TErr(("Failed to allocate object name string %s (len=%d).",
                          objName, strlen(objName) + 1));
                    rc = false;
                }
                else
                {
                    strcpy(cmdObj->objName, objName);
                }
            }

            if (rc == true)
            {
                for (int i = 0; varTable[i].varName != NULL; i++)
                {
                    if (varTable[i].varID != VARID_DATAPTR)
                    {
                        //
                        // The variable requires custom action.
                        //
                        varTable[i].pVarData = cmdHandler;
                    }
                }
                cmdObj->cmdHandler = cmdHandler;
                cmdObj->cmdTable = cmdTable;
                cmdObj->varTable = varTable;
                InsertTailList(&m_cmdObjList, &cmdObj->list);
            }
            else
            {
                SAFE_DELETE(cmdObj->objName);
                delete cmdObj;
            }
        }

        TExitMsg(("=%x", rc));
        return rc;
    }   //RegisterCmdHandler

    /**
     * This function unregisters a console command handler.
     *
     * @param cmdHandler Specifies the handler object to be unregistered.
     *
     * @return Returns true if the handler is successfully unregistered,
     *         false otherwise.
     */
    bool
    UnregisterCmdHandler(
        CmdHandler *cmdHandler
        )
    {
        bool rc = false;
        PCMD_OBJ cmdObj;

        TLevel(API);
        TEnterMsg(("handler=%p", cmdHandler));

        cmdObj = FindCmdObjByHandler(cmdHandler);
        if (cmdObj != NULL)
        {
            RemoveEntryList(&cmdObj->list);
            cmdObj->cmdHandler = NULL;
            SAFE_DELETE(cmdObj->objName);
            cmdObj->cmdTable = NULL;
            cmdObj->varTable = NULL;
            delete cmdObj;
            rc = true;
        }

        TExitMsg(("=%d", rc));
        return rc;
    }       //UnregisterCmdHandler

    /**
     * This function parses the command line into arguments and processes
     * the command.
     *
     * @param cmdLine Points to the command line.
     *
     * @return Success Returns ERR_SUCCESS.
     * @return Failure Returns error code.
     */
    int
    ParseCommand(
        char *cmdLine
        )
    {
        int rc = ERR_SUCCESS;
        static char sep[] = " \t";
        static char *apszArgs[MAX_NUM_ARGS];
        int cArgs = 0;
        char *psz;

        TLevel(FUNC);
        TEnterMsg(("Line=<%s>", cmdLine));
        //
        // Parse command line into arguments.
        //
        psz = strtok(cmdLine, sep);
        while ((psz != NULL) && (cArgs < MAX_NUM_ARGS))
        {
            apszArgs[cArgs] = psz;
            cArgs++;
            psz = strtok(NULL, sep);
        }

        if (cArgs > 0)
        {
            char *objName;
            char *cmdName;

            cmdName = strchr(apszArgs[0], '.');
            if (cmdName == NULL)
            {
                objName = NULL;
                cmdName = apszArgs[0];
            }
            else
            {
                objName = apszArgs[0];
                *cmdName = '\0';
                cmdName++;
            }

            rc = ProcessCommand(objName, cmdName, &apszArgs[1], cArgs - 1);
        }

        TExitMsg(("=%d", rc));
        return rc;
    }   //ParseCommand

};  //class ConsoleCommand

bool    g_fConsoleOn = false;
ConsoleCommand *ConsoleCommand::m_instance = NULL;

CMD_ENTRY ConsoleCommand::m_baseCmdTable[] =
{
    {"q",        BASECMD_QUIT,    "Exit console mode"},
    {"help",     BASECMD_HELP,    "List commands and info about them"},
    {"listobj",  BASECMD_LISTOBJ, "List registered command objects"},
    {"listvar",  BASECMD_LISTVAR, "List variables and info about them"},
    {"get",      BASECMD_GET,     "Get the value of a variable"},
    {"set",      BASECMD_SET,     "Set the value of a variable"},
    {NULL,       0,               NULL}
};

VAR_ENTRY ConsoleCommand::m_baseVarTable[] =
{
#ifdef _DBGTRACE_ENABLED
    {"TMod",     VARID_DATAPTR, VarInt32, &g_Trace.m_traceModules, 0, "0x%08x",
     "Bit mask of tracing modules."},
    {"TLevel",   VARID_DATAPTR, VarInt32, &g_Trace.m_traceLevel, 0, "%d",
     "Trace verbose level."},
    {"MsgLevel", VARID_DATAPTR, VarInt32, &g_Trace.m_msgLevel, 0, "%d",
     "Trace message verbose level."},
#endif
    {NULL,       0,             VarNone,  NULL, 0, NULL, NULL}
};

/**
 * This function registers a console command handler.
 *
 * @param objName Specifies the name of the object that provides the
 *        console command handler.
 * @param cmdTable Points to the command table to be registered for the
 *        object.
 * @param varTable Points to the variable table to be registered for the
 *        object.
 *
 * @return Returns true if the handler is successfully registered, false
 *         otherwise.
 */
bool
CmdHandler::RegisterCmdHandler(
    char      *objName,
    PCMD_ENTRY cmdTable,
    PVAR_ENTRY varTable
    )
{
    bool rc = false;
    ConsoleCommand *consoleCmd = ConsoleCommand::GetInstance();

    TLevel(API);
    TEnterMsg(("obj=%s,cmdTable=%p,varTable=%p", objName, cmdTable, varTable));

    if (consoleCmd != NULL)
    {
        rc = consoleCmd->RegisterCmdHandler(this, objName, cmdTable, varTable);
    }

    TExitMsg(("=%d", rc));
    return rc;
}   //RegisterCmdHandler

/**
 * This function unregisters a console command handler.
 *
 * @return Returns true if the handler is successfully unregistered, false
 *         otherwise.
 */
bool
CmdHandler::UnregisterCmdHandler(
    void
    )
{
    bool rc = false;
    ConsoleCommand *consoleCmd = ConsoleCommand::GetInstance();

    TLevel(API);
    TEnter();

    if (consoleCmd != NULL)
    {
        rc = consoleCmd->UnregisterCmdHandler(this);
    }

    TExitMsg(("=%d", rc));
    return rc;
}   //UnregisterCmdHandler

#ifdef __cplusplus
extern "C" {
#endif

/**
 * This function is called by the C-interpreter shell of the debug console
 * when "Console" is executed at the "->" prompt. The function will go into
 * an interactive loop reading command lines from the keyboard, parsing them
 * and either executing them or dispatching them to appropriate modules to
 * execute them.
 *
 * @return Returns the exit code ERR_EXIT.
 */
int
console(
    void
    )
{
    int rc = ERR_SUCCESS;
    static char cmdLine[128];
    ConsoleCommand *conCmd = ConsoleCommand::GetInstance();

    TLevel(API);
    TEnter();

    g_fConsoleOn = true;
    while (rc != ERR_EXIT)
    {
        ConPrintf(("\nConsole-> "));
        if (gets(cmdLine) != NULL)
        {
            rc = conCmd->ParseCommand(cmdLine);
        }
    }
    g_fConsoleOn = false;

    TExitMsg(("=%d", rc));
    return rc;
}   //console

#ifdef __cplusplus
}
#endif

#endif  //ifndef _CONSOLE_H
