/* $Id: drvPipePci.cpp 28800 2010-04-27 08:22:32Z vboxsync $ */
/** @file
 * Driver Pipe Pci.
 */

/*
 * Copyright (C) 2010 Max Rozhkov <feruxmax@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.

 * --------------------------------------------------------------------
 *
 * This code is based on:
 * DevE1000 - Intel 82540EM Ethernet Controller Emulation and others devices
 * drivers from VirtualBox OSE
 *
 * Copyright (C) 2009 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */


/*******************************************************************************
*   Header Files                                                               *
*******************************************************************************/
#define LOG_GROUP LOG_GROUP_MISC
#include <VBox/pdmdrv.h>
#include <VBox/version.h>
#include <VBox/err.h>
#include <VBox/log.h>

#include <iprt/assert.h>
#include <iprt/stream.h>
#include <iprt/uuid.h>
#include <iprt/file.h>
#include <iprt/alloc.h>
#include <iprt/string.h>
#include <iprt/semaphore.h>

#ifdef RT_OS_WINDOWS
# include <windows.h>
#else /* !RT_OS_WINDOWS */
# include <errno.h>
# include <unistd.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <sys/un.h>
#endif /* !RT_OS_WINDOWS */

/*******************************************************************************
*   Defined Constants And Macros                                               *
*******************************************************************************/

#define BUFSIZ                          4096

/** Converts a pointer to PDMDRVINS::IBase to a PPDMDRVINS. */
#define PDMIBASE_2_DRVINS(pInterface)   ( (PPDMDRVINS)((uintptr_t)pInterface - RT_OFFSETOF(PDMDRVINS, IBase)) )
/** Converts a pointer to DRVNAMEDPIPE::IMedia to a PDRVNAMEDPIPE. */
#define PDMISTREAM_2_DRVPIPEPCI(pInterface) ( (pdrvPipePci)((uintptr_t)pInterface - RT_OFFSETOF(drvPipePci, IStream)) )

/*******************************************************************************
*   Structures and Typedefs                                                    *
*******************************************************************************/

/**
 * driver Instance Data.
 */
typedef struct drvPipePci
{
    /** The stream interface. */
    PDMISTREAM          IStream;
    /** Pointer to the driver instance. */
    PPDMDRVINS          pDrvIns;
    /** Pointer to the named pipe file name. (Freed by MM) */
    char                *pszLocation;
    /** Flag whether VirtualBox represents the server or client side. */
    bool                fIsServer;
#ifdef RT_OS_WINDOWS
    /** File handle of the named pipe. */
    HANDLE              NamedPipe;
    /** Overlapped structure for writes. */
    OVERLAPPED          OverlappedWrite;
    /** Overlapped structure for reads. */
    OVERLAPPED          OverlappedRead;
    /** Listen thread wakeup semaphore */
    RTSEMEVENTMULTI     ListenSem;
#else /* !RT_OS_WINDOWS */
    /** Socket handle of the local socket for server. */
    int                 LocalSocketServer;
    /** Socket handle of the local socket. */
    int                 LocalSocket;
    /** Thread for listening for new connections. */
    RTTHREAD            ListenThread;
#endif /* !RT_OS_WINDOWS */
    /** Flag to signal listening thread to shut down. */
    bool volatile       fShutdown;
} drvPipePci;
typedef drvPipePci *pdrvPipePci;

    
/*******************************************************************************
*   Internal Functions                                                         *
*******************************************************************************/
static DECLCALLBACK(int) PipeListenLoop(RTTHREAD ThreadSelf, void *pvUser);
static void PipeShutdownListener(pdrvPipePci pThis);
static DECLCALLBACK(int) PipeRecvThread(RTTHREAD ThreadSelf, void *pvUser);


/* -=-=-=-=- IBase -=-=-=-=- */
    
    /**
     * @interface_method_impl{PDMIBASE,pfnQueryInterface}
     */
    static DECLCALLBACK(void *) drvPipePciQueryInterface(PPDMIBASE pInterface, const char *pszIID)
    {
        PPDMDRVINS          pDrvIns = PDMIBASE_2_PDMDRV(pInterface);
        pdrvPipePci    pThis   = PDMINS_2_DATA(pDrvIns, pdrvPipePci);
        
        PDMIBASE_RETURN_INTERFACE(pszIID, PDMIBASE, &pDrvIns->IBase);
        PDMIBASE_RETURN_INTERFACE(pszIID, PDMISTREAM, &pThis->IStream);
        return NULL;
    } 
    
    static DECLCALLBACK(int) drvPipePciRead(PPDMISTREAM pInterface, void *pvBuf, size_t *pcbRead)
    {
        int rc = VINF_SUCCESS;
        pdrvPipePci pThis = PDMISTREAM_2_DRVPIPEPCI(pInterface);
#ifdef RT_OS_WINDOWS
        while(pThis->NamedPipe == INVALID_HANDLE_VALUE)
            RTThreadSleep(1000);
        
        DWORD cbReallyRead;
        pThis->OverlappedRead.Offset     = 0;
        pThis->OverlappedRead.OffsetHigh = 0;
        if (!ReadFile(pThis->NamedPipe, pvBuf,(size_t)BUFSIZ, &cbReallyRead, &pThis->OverlappedRead))
        {
            DWORD uError = GetLastError();
            
            if (   uError == ERROR_PIPE_LISTENING
                   || uError == ERROR_PIPE_NOT_CONNECTED)
            {
                /* No connection yet/anymore */
                cbReallyRead = 0;
                
                /* wait a bit or else we'll be called right back. */
                RTThreadSleep(100);
            }
            else
            {
                if (uError == ERROR_IO_PENDING)
                {
                    uError = 0;
                    
                    /* Wait for incoming bytes. */
                    if (GetOverlappedResult(pThis->NamedPipe, &pThis->OverlappedRead, &cbReallyRead, TRUE) == FALSE)
                        uError = GetLastError();
                }
                
                rc = RTErrConvertFromWin32(uError);
                Log(("drvNamedPipeRead: ReadFile returned %d (%Rrc)\n", uError, rc));
            }
        }
        
        if (RT_FAILURE(rc))
        {
            Log(("drvNamedPipeRead: FileRead returned %Rrc fShutdown=%d\n", rc, pThis->fShutdown));
            if (    !pThis->fShutdown
                    &&  (   rc == VERR_EOF
                            || rc == VERR_BROKEN_PIPE
                            )
                    )
            {
                FlushFileBuffers(pThis->NamedPipe);
                DisconnectNamedPipe(pThis->NamedPipe);
                if (!pThis->fIsServer)
                {
                    CloseHandle(pThis->NamedPipe);
                    pThis->NamedPipe = INVALID_HANDLE_VALUE;
                }
                /* pretend success */
                rc = VINF_SUCCESS;
            }
            cbReallyRead = 0;
        }
        *pcbRead = (size_t)cbReallyRead;
#else /* !RT_OS_WINDOWS */
        ssize_t cbReallyRead;
        while (pThis->LocalSocket == -1)
            RTThreadSleep(1000);
        
        cbReallyRead = recv(pThis->LocalSocket, pvBuf, (size_t)BUFSIZ, 0);
        if (cbReallyRead == 0)
        {
            int tmp = pThis->LocalSocket;
            pThis->LocalSocket = -1;
            close(tmp);
        }
        else if (cbReallyRead == -1)
        {
            cbReallyRead = 0;
            rc = RTErrConvertFromErrno(errno);
        }
        *pcbRead = cbReallyRead;
#endif /* !RT_OS_WINDOWS */

        LogFlow(("%s: *pcbRead=%zu returns %Rrc\n", __FUNCTION__, *pcbRead, rc));
        return rc;
    }
    
    static DECLCALLBACK(int) drvPipePciWrite(PPDMISTREAM pInterface, const void *pvBuf, size_t *pcbWrite)
    {
        int rc = VINF_SUCCESS;
        pdrvPipePci pThis = PDMISTREAM_2_DRVPIPEPCI(pInterface);
           
#ifdef RT_OS_WINDOWS
            if (pThis->NamedPipe != INVALID_HANDLE_VALUE)
            {
                DWORD cbWritten = (DWORD)*pcbWrite;
                pThis->OverlappedWrite.Offset     = 0;
                pThis->OverlappedWrite.OffsetHigh = 0;
                if (!WriteFile(pThis->NamedPipe, pvBuf, cbWritten, NULL, &pThis->OverlappedWrite))
                {
                    DWORD uError = GetLastError();
                    
                    if (   uError == ERROR_PIPE_LISTENING
                           || uError == ERROR_PIPE_NOT_CONNECTED)
                    {
                        /* No connection yet/anymore; just discard the write (pretening everything was written). */;
                    }
                    else if (uError != ERROR_IO_PENDING)
                    {
                        rc = RTErrConvertFromWin32(uError);
                        Log(("drvNamedPipeWrite: WriteFile returned %d (%Rrc)\n", uError, rc));
                        cbWritten = 0;
                    }
                    else
                    {
                        /* Wait for the write to complete. */
                        if (GetOverlappedResult(pThis->NamedPipe, &pThis->OverlappedWrite, &cbWritten, TRUE /*bWait*/) == FALSE)
                            rc = RTErrConvertFromWin32(uError = GetLastError());
                    }
                }
                
                if (RT_FAILURE(rc))
                {
                    /** @todo WriteFile(pipe) has been observed to return  ERROR_NO_DATA
                     *        (VERR_NO_DATA) instead of ERROR_BROKEN_PIPE, when the pipe is
                     *        disconnected. */
                    if (    rc == VERR_EOF
                            ||  rc == VERR_BROKEN_PIPE)
                    {
                        FlushFileBuffers(pThis->NamedPipe);
                        DisconnectNamedPipe(pThis->NamedPipe);
                        if (!pThis->fIsServer)
                        {
                            CloseHandle(pThis->NamedPipe);
                            pThis->NamedPipe = INVALID_HANDLE_VALUE;
                        }
                        /* pretend success */
                        rc = VINF_SUCCESS;
                    }
                    cbWritten = 0;
                }
            }
#else /* !RT_OS_WINDOWS */
            if (pThis->LocalSocket != -1)
            {
                ssize_t cbWritten;
                cbWritten = send(pThis->LocalSocket, pvBuf, *pcbWrite, 0);
                if (cbWritten == 0)
                {
                    int tmp = pThis->LocalSocket;
                    pThis->LocalSocket = -1;
                    close(tmp);
                }
                else if (cbWritten == -1)
                {
                    cbWritten = 0;
                    rc = RTErrConvertFromErrno(errno);
                }
            }
#endif /* !RT_OS_WINDOWS */
            
        return rc;
    }
    
static DECLCALLBACK(void) drvPipePciDestruct(PPDMDRVINS pDrvIns)
{
    pdrvPipePci pThis = PDMINS_2_DATA(pDrvIns, pdrvPipePci);
    /*
     * Check the versions here as well since the destructor is *always* called.
    */
    PipeShutdownListener(pThis);

    /*
     * While the thread exits, clean up as much as we can.
     */
#ifdef RT_OS_WINDOWS
    if (pThis->NamedPipe != INVALID_HANDLE_VALUE)
    {
        CloseHandle(pThis->NamedPipe);
        pThis->NamedPipe = INVALID_HANDLE_VALUE;
    }
    if (pThis->OverlappedRead.hEvent != NULL)
    {
        CloseHandle(pThis->OverlappedRead.hEvent);
        pThis->OverlappedRead.hEvent = NULL;
    }
    if (pThis->OverlappedWrite.hEvent != NULL)
    {
        CloseHandle(pThis->OverlappedWrite.hEvent);
        pThis->OverlappedWrite.hEvent = NULL;
    }
#else /* !RT_OS_WINDOWS */
    Assert(pThis->LocalSocketServer == -1);
    if (pThis->LocalSocket != -1)
    {
        int rc = shutdown(pThis->LocalSocket, SHUT_RDWR);
        AssertRC(rc == 0); NOREF(rc);

        rc = close(pThis->LocalSocket);
        Assert(rc == 0);
        pThis->LocalSocket = -1;
    }
    if (   pThis->fIsServer
        && pThis->pszLocation)
        RTFileDelete(pThis->pszLocation);
    
    /*
     * Wait for the thread.
     */
    if (pThis->ListenThread != NIL_RTTHREAD)
    {
        int rc = RTThreadWait(pThis->ListenThread, 30000, NULL);
        if (RT_SUCCESS(rc))
            pThis->ListenThread = NIL_RTTHREAD;
        else
            LogRel(("NamedPipe%d: listen thread did not terminate (%Rrc)\n", pDrvIns->iInstance, rc));
    }
#endif /* !RT_OS_WINDOWS */

    /*
     * The last bits of cleanup.
     */
#ifdef RT_OS_WINDOWS
    if (pThis->ListenSem != NIL_RTSEMEVENT)
    {
        RTSemEventMultiDestroy(pThis->ListenSem);
        pThis->ListenSem = NIL_RTSEMEVENT;
    }
#endif
    return;
}


static DECLCALLBACK(int) drvPipePciConstruct(PPDMDRVINS pDrvIns, PCFGMNODE pCfg, uint32_t iInstance)
{
    /*
     * Check that the Drvice instance and Drvice helper structures are compatible.
     */
    AssertLogRelMsgReturn(pDrvIns->u32Version            == PDM_DRVINS_VERSION, ("%#x, expected %#x\n", pDrvIns->u32Version,            PDM_DRVINS_VERSION), VERR_VERSION_MISMATCH);
    AssertLogRelMsgReturn(pDrvIns->pHlpR3->u32Version == PDM_DRVHLPR3_VERSION, ("%#x, expected %#x\n", pDrvIns->pHlpR3->u32Version, PDM_DRVHLPR3_VERSION), VERR_VERSION_MISMATCH);
    int rc = VINF_SUCCESS;
    /*
     * Initialize the instance data so that the destructure won't mess up.
     */
    pdrvPipePci pThis = PDMINS_2_DATA(pDrvIns, pdrvPipePci);
   
    /*
     * Init the static parts.
     */
    pThis->pDrvIns                      = pDrvIns;
#ifdef RT_OS_WINDOWS
    pThis->fIsServer                    = false;
    pThis->NamedPipe                    = INVALID_HANDLE_VALUE;
    pThis->ListenSem                    = NIL_RTSEMEVENTMULTI;
    pThis->OverlappedWrite.hEvent       = NULL;
    pThis->OverlappedRead.hEvent        = NULL;
    pThis->pszLocation                  = "\\\\.\\pipe\\vmwaredebug";
#else /* !RT_OS_WINDOWS */
    pThis->fIsServer                    = true;
    pThis->LocalSocketServer            = -1;
    pThis->LocalSocket                  = -1;
    pThis->pszLocation                  = "/tmp/pci.pipe";/** @todo make config opt for this */
    pThis->ListenThread                 = NIL_RTTHREAD;
#endif /* !RT_OS_WINDOWS */
    pThis->fShutdown                    = false;
    /* IBase. */
    pDrvIns->IBase.pfnQueryInterface = drvPipePciQueryInterface;

    pThis->IStream.pfnWrite = drvPipePciWrite;
    pThis->IStream.pfnRead = drvPipePciRead;

    /*
     * Create/Open the pipe.
     */
#ifdef RT_OS_WINDOWS
    /* Connect to the named pipe. */
    pThis->NamedPipe = CreateFile(pThis->pszLocation, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                                  OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
    if (pThis->NamedPipe == INVALID_HANDLE_VALUE)
    {
        rc = RTErrConvertFromWin32(GetLastError());
        return PDMDrvHlpVMSetError(pDrvIns, rc, RT_SRC_POS, N_("NamedPipe#%d failed to connect to named pipe %s"),
                                   pDrvIns->iInstance, pThis->pszLocation);
    }
    
    memset(&pThis->OverlappedWrite, 0, sizeof(pThis->OverlappedWrite));
    pThis->OverlappedWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    AssertReturn(pThis->OverlappedWrite.hEvent != NULL, VERR_OUT_OF_RESOURCES);
    
    memset(&pThis->OverlappedRead, 0, sizeof(pThis->OverlappedRead));
    pThis->OverlappedRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    AssertReturn(pThis->OverlappedRead.hEvent != NULL, VERR_OUT_OF_RESOURCES);
    
    
#else /* !RT_OS_WINDOWS */
    int s = socket(PF_UNIX, SOCK_STREAM, 0);
    if (s == -1)
        return PDMDrvHlpVMSetError(pDrvIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                   N_("NamedPipe#%d failed to create local socket"), pDrvIns->iInstance);
    
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, pThis->pszLocation, sizeof(addr.sun_path) - 1);
    
    /* Bind address to the local socket. */
    pThis->LocalSocketServer = s;
    RTFileDelete(pThis->pszLocation);
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) == -1)
        return PDMDrvHlpVMSetError(pDrvIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                   N_("NamedPipe#%d failed to bind to local socket %s"),
                                   pDrvIns->iInstance, pThis->pszLocation);
    rc = RTThreadCreate(&pThis->ListenThread, PipeListenLoop, (void *)pThis, 0,
                        RTTHREADTYPE_IO, RTTHREADFLAGS_WAITABLE, "PciPipe");
    if (RT_FAILURE(rc))
        return PDMDrvHlpVMSetError(pDrvIns, rc,  RT_SRC_POS,
                                   N_("NamedPipe#%d failed to create listening thread"), pDrvIns->iInstance);
    
   #endif /* !RT_OS_WINDOWS */

    LogRel(("NamedPipe: location %s, %s\n", pThis->pszLocation, pThis->fIsServer ? "server" : "client"));
    
    return VINF_SUCCESS;
}

/* -=-=-=-=- listen thread -=-=-=-=- */

/**
 * Receive thread loop.
 *
 * @returns 0 on success.
 * @param   ThreadSelf  Thread handle to this thread.
 * @param   pvUser      User argument.
 */
static DECLCALLBACK(int) PipeListenLoop(RTTHREAD ThreadSelf, void *pvUser)
{
    pdrvPipePci pThis = (pdrvPipePci)pvUser;
    int             rc = VINF_SUCCESS;
#ifndef RT_OS_WINDOWS
    while (RT_LIKELY(!pThis->fShutdown))
    {
        if (listen(pThis->LocalSocketServer, 0) == -1)
        {
            rc = RTErrConvertFromErrno(errno);
            LogRel(("NamedPipe%d: listen failed, rc=%Rrc\n", pThis->pDrvIns->iInstance, rc));
            break;
        }
        int s = accept(pThis->LocalSocketServer, NULL, NULL);
        if (s == -1)
        {
            rc = RTErrConvertFromErrno(errno);
            LogRel(("NamedPipe%d: accept failed, rc=%Rrc\n", pThis->pDrvIns->iInstance, rc));
            break;
        }
        if (pThis->LocalSocket != -1)
        {
            LogRel(("NamedPipe%d: only single connection supported\n", pThis->pDrvIns->iInstance));
            close(s);
        }
        else
            pThis->LocalSocket = s;
    }
#endif /* !RT_OS_WINDOWS */

    return VINF_SUCCESS;
}

/* -=-=-=-=- PDMDevREG -=-=-=-=- */

/**
 * Common worker for DevNamedPipePowerOff and DevNamedPipeDestructor.
 *
 * @param   pThis               The instance data.
 */
static void PipeShutdownListener(pdrvPipePci pThis)
{
    /*
     * Signal shutdown of the listener thread.
     */
    pThis->fShutdown = true;
#ifdef RT_OS_WINDOWS
    if (    pThis->fIsServer
        &&  pThis->NamedPipe != INVALID_HANDLE_VALUE)
    {
        FlushFileBuffers(pThis->NamedPipe);
        DisconnectNamedPipe(pThis->NamedPipe);

        BOOL fRc = CloseHandle(pThis->NamedPipe);
        Assert(fRc); NOREF(fRc);
        pThis->NamedPipe = INVALID_HANDLE_VALUE;

        /* Wake up listen thread */
        if (pThis->ListenSem != NIL_RTSEMEVENT)
            RTSemEventMultiSignal(pThis->ListenSem);
    }
#else
    if (    pThis->fIsServer
        &&  pThis->LocalSocketServer != -1)
    {
        int rc = shutdown(pThis->LocalSocketServer, SHUT_RDWR);
        AssertRC(rc == 0); NOREF(rc);

        rc = close(pThis->LocalSocketServer);
        AssertRC(rc == 0);
        pThis->LocalSocketServer = -1;
    }
#endif
}

/**
 * The Drvice registration structure.
 */
static const PDMDRVREG g_drvPipePci =
{
    /* u32Version */
    PDM_DRVREG_VERSION,
    /* szName */
    "PipePci",
    /* szRCMod */
    "",
    /* szR0Mod */
    "",
    /* pszDescription */
    "driver Pipe Pci",
    /* fFlags */
    PDM_DRVREG_FLAGS_HOST_BITS_DEFAULT,
    /* fClass */
    PDM_DRVREG_CLASS_STREAM,
    /* cMaxInstances */
    ~0,
    /* cbInstance */
    sizeof(drvPipePci),
    /* pfnConstruct */
    drvPipePciConstruct,
    /* pfnDestruct */
    drvPipePciDestruct,
    /* pfnRelocate */
    NULL,
    /* pfnIOCtl */
    NULL,
    /* pfnPowerOn */
    NULL,
    /* pfnReset */
    NULL,
    /* pfnSuspend */
    NULL,
    /* pfnResume */
    NULL,
    /* pfnAttach */
    NULL,
    /* pfnDetach */
    NULL,
    /* pfnPowerOff */
    NULL,
    /* pfnSoftReset */
    NULL,
    /* u32VersionEnd */
    PDM_DRVREG_VERSION
};


/**
 * Register builtin Drvices.
 *
 * @returns VBox status code.
 * @param   pCallbacks      Pointer to the callback table.
 * @param   u32Version      VBox version number.
 */
extern "C" DECLEXPORT(int) VBoxDriversRegister(PPDMDRVREGCB pCallbacks, uint32_t u32Version)
{
    LogFlow(("drvPipePci::VBoxDdriverRegister: u32Version=%#x pCallbacks->u32Version=%#x\n", u32Version, pCallbacks->u32Version));

    AssertLogRelMsgReturn(pCallbacks->u32Version == PDM_DRVREG_CB_VERSION,
                          ("%#x, expected %#x\n", pCallbacks->u32Version, PDM_DRVREG_CB_VERSION),
                          VERR_VERSION_MISMATCH);

    return pCallbacks->pfnRegister(pCallbacks, &g_drvPipePci);
}


