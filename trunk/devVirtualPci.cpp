/* $Id: VBoxSampleDevice.cpp 28800 2010-04-27 08:22:32Z vboxsync $ */
/** @file
 * Virtual Pci Device.
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
#include <VBox/pdmdev.h>
#include <VBox/version.h>
#include <VBox/err.h>
#include <VBox/log.h>

#include <iprt/assert.h>
/*for PCI???*/
#include <VBox/param.h>
#include <VBox/tm.h>
#include <VBox/vm.h>

/*for pipe*/
#ifdef RT_OS_WINDOWS
# include <windows.h>
#else /* !RT_OS_WINDOWS */
# include <errno.h>
# include <unistd.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <sys/un.h>
#endif /* !RT_OS_WINDOWS */

#include <iprt/assert.h>
#include <iprt/file.h>
#include <iprt/stream.h>
#include <iprt/alloc.h>
#include <iprt/string.h>
#include <iprt/semaphore.h>
#include <iprt/uuid.h>

/*******************************************************************************
*   Defined Constants And Macros                                               *
*******************************************************************************/
/* The size of register area mapped to I/O space */
#define E1K_IOPORT_SIZE                 0x8
/* The size of memory-mapped register area */
#define E1K_MM_SIZE                     0x20000
#define BUFSIZ                          4096
/* PCI device regs */
#define STATUS_REG                      0x0
#define COMAND_REG                      0x0
#define START_ADR_REG                   0x1
#define SIZE_REG                        0x2
#define DATA_REG                        0x3
#define REG_IS_RESERVED                 0xf0       //возвращается при чтении из зарезервированного регистра
/*PCI comands */
#define WRONG_CMD                       0x00000000
#define WRITE_CMD                       0x00000001 //записать содержимое буфера приёма в память
#define READ_CMD                        0x00000002 //считать в буфер отправки из памяти
#define SEND_CMD                        0x00000003 //отправить содержимое буфера отправки
#define START_READ_CMD                  0x00000004 //начать приём пакетов => вкл. прерывания
#define SET_IRQ                         0x00000005 //активировать линию прерывания
#define RESET_IRQ                       0x00000006 //деактивировать линию прерывания
/*status */
#define DMA_READ_DONE                   0x00000001
#define DMA_WRITE_DONE                  0x00000002
#define DATA_RECVD                      0x00000003
#define BUFF_IS_EMPTY                   0x00000004
#define IRQ_ATIVED                      0x00000005 //лирия прерывания активирована командой SET_IRQ

#define PDMIBASE_2_VIRTUALPCISTATE(pInstance)       ( (virtualPciState *)((uintptr_t)(pInterface) - RT_OFFSETOF(virtualPciState, IBase)) )

/*******************************************************************************
*   Structures and Typedefs                                                    *
*******************************************************************************/

/**
 * Device Instance Data.
 */
typedef struct virtualPciState
{
    /** Base address of memory-mapped registers. */
    RTGCPHYS    addrMMReg;
    /** Base port of I/O space region. */
    RTIOPORT    addrIOPort;
    /** EMT: */
    PCIDEVICE   pciDevice;
    PPDMDEVINS pDevIns;

    uint32_t    baseAddr;
    uint32_t    bufSize;
    uint32_t    size_send;
    uint32_t    size_recv;
    uint32_t    status_reg;
    char        send_buf[BUFSIZ];
    char        recv_buf[BUFSIZ];
    char *pRcvPointer;
    
    /** Flag to notify the dma thread it should terminate. */
    volatile bool               DMAon;
    /** The event semaphore the thread is waiting on during suspended I/O. */
    RTSEMEVENT          SuspendDMASem;
    RTSEMEVENT          SuspendReadSem;
    RTTHREAD            AsyncDMAThread;
    uint32_t    dma_cmd;
    /*---------------------Driver------------------------------------------*/
    /** Pointer to the attached base driver. */
    R3PTRTYPE(PPDMIBASE)            pDrvBase;
    /** LUN\#0: The base interface. */
    PDMIBASE                        IBase;
    /** Pointer to the stream interface of the driver below us. */
    PPDMISTREAM                 pDrvStream;
    RTTHREAD            RecvData;
    /** Flag to notify the receive thread it should terminate. */
    volatile bool               fShutdown;
   } virtualPciState;
typedef virtualPciState *pVirtualPciState;

/*******************************************************************************
*   Internal Functions                                                         *
*******************************************************************************/

static DECLCALLBACK(void) e1kConfigurePCI(PCIDEVICE* pci);
static DECLCALLBACK(int) e1kMap(PPCIDEVICE pPciDev, int iRegion,
                                RTGCPHYS GCPhysAddress, uint32_t cb, PCIADDRESSSPACE enmType);
PDMBOTHCBDECL(int) VirtualPciPortRead(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port,
                                      uint32_t *pu32, unsigned cb);
PDMBOTHCBDECL(int) VirtualPciPortWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port,
                                       uint32_t u32, unsigned cb);
PDMBOTHCBDECL(int) VirtualPciMMIORead(PPDMDEVINS pDevIns, void *pvUser,
                               RTGCPHYS GCPhysAddr, void *pv, unsigned cb);
PDMBOTHCBDECL(int) VirtualPciMMIOWrite(PPDMDEVINS pDevIns, void *pvUser,
                                RTGCPHYS GCPhysAddr, void *pv, unsigned cb);
static DECLCALLBACK(void *) VirtualPciQueryInterface(PPDMIBASE pInterface, const char *pszIID);
static DECLCALLBACK(int) RecvDataThread(RTTHREAD ThreadSelf, void *pvUser);
static DECLCALLBACK(int) pciAsyncDMALoop(RTTHREAD ThreadSelf, void *pvUser);
        

/**
 * @interface_method_impl{PDMIBASE, pfnQueryInterface}
 */
static DECLCALLBACK(void *) VirtualPciQueryInterface(PPDMIBASE pInterface, const char *pszIID)
{
    virtualPciState *pThis = PDMIBASE_2_VIRTUALPCISTATE(pInterface);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIBASE, &pThis->IBase);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMISTREAM, pThis->pDrvStream);
    return NULL;
}

/** Asynch I/O thread for an interface DMA.  */
static DECLCALLBACK(int) pciAsyncDMALoop(RTTHREAD ThreadSelf, void *pvUser)
{
    int             rc   = VINF_SUCCESS;
    pVirtualPciState pThis = (pVirtualPciState)pvUser;

    while (!pThis->DMAon){
        rc = RTSemEventWait(pThis->SuspendDMASem, RT_INDEFINITE_WAIT);
        switch(pThis->dma_cmd){
        case READ_CMD:
            PDMDevHlpPhysRead(pThis->pDevIns, pThis->baseAddr, pThis->send_buf,
                                    pThis->size_send < BUFSIZ? pThis->size_send:BUFSIZ);
            pThis->status_reg = DMA_READ_DONE;
            PDMDevHlpPCISetIrq(pThis->pDevIns, 0, 1);
            break;
        case WRITE_CMD:
            PDMDevHlpPhysWrite(pThis->pDevIns, pThis->baseAddr, pThis->recv_buf,
                                    pThis->size_recv < BUFSIZ? pThis->size_recv:BUFSIZ);
            pThis->status_reg = DMA_WRITE_DONE;
            PDMDevHlpPCISetIrq(pThis->pDevIns, 0, 1); // должен успеть обработать прерывание до
            rc = RTSemEventSignal(pThis->SuspendReadSem);//прихода нового паека данных
            break;
        default:
            ;
        }
        pThis->dma_cmd = WRONG_CMD;
    }
    return rc;
}

static DECLCALLBACK(int) RecvDataThread(RTTHREAD ThreadSelf, void *pvUser){
    int rc = VINF_SUCCESS;
    pVirtualPciState pThis = (pVirtualPciState)pvUser;
    while (!pThis->fShutdown)
    {
        rc = RTSemEventWait(pThis->SuspendReadSem, RT_INDEFINITE_WAIT);
        pThis->pDrvStream->pfnRead(pThis->pDrvStream, pThis->recv_buf, &pThis->size_recv);
        if(pThis->size_recv>0){
            pThis->pRcvPointer = pThis->recv_buf;
            pThis->status_reg = DATA_RECVD;
            PDMDevHlpPCISetIrq(pThis->pDevIns, 0, 1);
        }
    } 
    return rc;
}

static DECLCALLBACK(void) e1kConfigurePCI(PCIDEVICE* pci)
{
    /* The PCI devices configuration. */
    PCIDevSetVendorId(pci,   0x1af4);	/* PCI vendor, same as Cam Macdonell used for KVM,offset==0x00 */
    PCIDevSetDeviceId(pci,   0x1110);	/* PCI device ID, same as Cam Macdonell used for KVM,offset==0x02 */

    PCIDevSetSubSystemVendorId(pci,   0x1af4);	/* PCI sub vendor, offset==0x2C */
    PCIDevSetSubSystemId(pci,   0x1110);	/* PCI sub device ID, offset==0x02 */

    PCIDevSetClassBase(pci,   0x05);		/* Memory controller,  offset==0x0B */
    PCIDevSetClassSub(pci,   0x00);		/* Ram controller, offset==0x0A */
    PCIDevSetClassProg(pci,   0x00);		/* Ram controller, offset==0x09 */

    PCIDevSetHeaderType(pci,   0x00);		/* Header type, offset==0x0E */

    pci->config[VBOX_PCI_STATUS] = 0x30;
    pci->config[VBOX_PCI_STATUS+1] = 0x02;
    PCIDevSetInterruptPin(pci,   0x01);

    pci->config[VBOX_PCI_BASE_ADDRESS_0] = 0x01;
    pci->config[VBOX_PCI_BASE_ADDRESS_1] = 0x00;
}

/**
 * Map PCI I/O region.
 *
 * @return  VBox status code.
 * @param   pPciDev         Pointer to PCI device. Use pPciDev->pDevIns to get the device instance.
 * @param   iRegion         The region number.
 * @param   GCPhysAddress   Physical address of the region. If iType is PCI_ADDRESS_SPACE_IO, this is an
 *                          I/O port, else it's a physical address.
 *                          This address is *NOT* relative to pci_mem_base like earlier!
 * @param   cb              Region size.
 * @param   enmType         One of the PCI_ADDRESS_SPACE_* values.
 * @thread  EMT
 */
static DECLCALLBACK(int) e1kMap(PPCIDEVICE pPciDev, int iRegion,
                                RTGCPHYS GCPhysAddress, uint32_t cb, PCIADDRESSSPACE enmType)
{
    int       rc;
    pVirtualPciState pThis = PDMINS_2_DATA(pPciDev->pDevIns, pVirtualPciState);

    switch (enmType)
    {
        case PCI_ADDRESS_SPACE_IO:
            pThis->addrIOPort = (RTIOPORT)GCPhysAddress;
            rc = PDMDevHlpIOPortRegister(pPciDev->pDevIns, pThis->addrIOPort, cb, 0,
                                         VirtualPciPortWrite, VirtualPciPortRead, NULL, NULL, "VirtPCI");
            break;
        case PCI_ADDRESS_SPACE_MEM:
            pThis->addrMMReg = GCPhysAddress;
            rc = PDMDevHlpMMIORegister(pPciDev->pDevIns, GCPhysAddress, cb, 0,
                                       VirtualPciMMIOWrite, VirtualPciMMIORead, NULL, "VirtPCI");
            break;
        default:
            /* We should never get here */
            AssertMsgFailed(("Invalid PCI address space param in map callback"));
            rc = VERR_INTERNAL_ERROR;
            break;
    }
    return rc;
}

/**
 * Port I/O Handler for OUT operations.
 *
 * @returns VBox status code.
 *
 * @param   pDevIns     The device instance.
 * @param   pvUser      User argument.
 * @param   Port        Port number used for the IN operation.
 * @param   u32         The value to output.
 * @param   cb          The value size in bytes.
 */
PDMBOTHCBDECL(int) VirtualPciPortWrite(PPDMDEVINS pDevIns, void *pvUser,
                                       RTIOPORT Port, uint32_t u32, unsigned cb)
{
    pVirtualPciState pThis = PDMINS_2_DATA(pDevIns, pVirtualPciState);
    int            rc = VINF_SUCCESS;
    size_t cbProcessed = 1;
    
    Port-=pThis->addrIOPort;
    switch(Port){
    case START_ADR_REG:
        pThis->baseAddr=u32;
        break;
    case SIZE_REG:
        pThis->size_send=u32;
        break;
    case COMAND_REG:
        if(u32 == READ_CMD){
            pThis->dma_cmd = READ_CMD;
            rc = RTSemEventSignal(pThis->SuspendDMASem);
        }
        else if(u32 == WRITE_CMD){
            pThis->dma_cmd = WRITE_CMD;
            rc = RTSemEventSignal(pThis->SuspendDMASem);
        }
        else if(u32 == SEND_CMD){
            cbProcessed = pThis->size_send;
            pThis->pDrvStream->pfnWrite(pThis->pDrvStream, pThis->send_buf, &cbProcessed);
        }
        else if(u32 == START_READ_CMD){
            rc = RTSemEventSignal(pThis->SuspendReadSem);
        }
        else if(u32 == SET_IRQ){
            pThis->status_reg = IRQ_ATIVED;
            PDMDevHlpPCISetIrq(pThis->pDevIns, 0, 1);
        }
        else if(u32 == RESET_IRQ){
             pThis->status_reg = 0;
             PDMDevHlpPCISetIrq(pThis->pDevIns, 0, 0);
        }
        break;
    case DATA_REG:
        pThis->pDrvStream->pfnWrite(pThis->pDrvStream, &u32, &cbProcessed);
        break;
    default:
        ;
    } 
    return rc;
}

/**
 * Port I/O Handler for IN operations.
 *
 * @returns VBox status code.
 *
 * @param   pDevIns     The device instance.
 * @param   pvUser      User argument.
 * @param   Port        Port number used for the IN operation.
 * @param   pu32        Where to return the read value.
 * @param   cb          The value size in bytes.
 */
PDMBOTHCBDECL(int) VirtualPciPortRead(PPDMDEVINS pDevIns, void *pvUser,
                                      RTIOPORT Port, uint32_t *pu32, unsigned cb)
{
    pVirtualPciState pThis = PDMINS_2_DATA(pDevIns, pVirtualPciState);
    int            rc = VINF_SUCCESS;

    PDMDevHlpPCISetIrq(pThis->pDevIns, 0, 0);
    Port-=pThis->addrIOPort;
    switch(Port){
    case STATUS_REG:
        *pu32 = pThis->status_reg;
        break;
    case SIZE_REG:
        *pu32 = pThis->size_recv;
        break;
    case DATA_REG:
        if(pThis->size_recv){
            *pu32 = *(pThis->pRcvPointer++);
            pThis->size_recv--;
        }else{
            *pu32=0;
            pThis->status_reg=BUFF_IS_EMPTY;
            rc = RTSemEventSignal(pThis->SuspendReadSem);
        }
        break;
    default:
        *pu32=REG_IS_RESERVED;
    }

    return rc;
}
/**
 * I/O handler for memory-mapped read operations.
 *
 * @returns VBox status code.
 *
 * @param   pDevIns     The device instance.
 * @param   pvUser      User argument.
 * @param   GCPhysAddr  Physical address (in GC) where the read starts.
 * @param   pv          Where to store the result.
 * @param   cb          Number of bytes read.
 * @thread  EMT
 */
PDMBOTHCBDECL(int) VirtualPciMMIORead(PPDMDEVINS pDevIns, void *pvUser,
                               RTGCPHYS GCPhysAddr, void *pv, unsigned cb)
{
    int       rc;
    pVirtualPciState pThis = PDMINS_2_DATA(pDevIns, pVirtualPciState);
    GCPhysAddr-=pThis->addrMMReg;
    if(GCPhysAddr<BUFSIZ){
        if(cb==1)
            *(char*)pv=pThis->recv_buf[GCPhysAddr];
        else if(cb==2)
            *(short*)pv=*(short*)&(pThis->recv_buf[GCPhysAddr]);
        else
            *(int*)pv=*(int*)&(pThis->recv_buf[GCPhysAddr]);
    }else{
        if(cb==1)
            *(char*)pv=pThis->send_buf[GCPhysAddr];
        else if(cb==2)
            *(short*)pv=*(short*)&(pThis->send_buf[GCPhysAddr]);
        else
            *(int*)pv=*(int*)&(pThis->send_buf[GCPhysAddr]);
    }
    rc = 0;
    return rc;
}

/**
 * Memory mapped I/O Handler for write operations.
 *
 * @returns VBox status code.
 *
 * @param   pDevIns     The device instance.
 * @param   pvUser      User argument.
 * @param   GCPhysAddr  Physical address (in GC) where the read starts.
 * @param   pv          Where to fetch the value.
 * @param   cb          Number of bytes to write.
 * @thread  EMT
 */
PDMBOTHCBDECL(int) VirtualPciMMIOWrite(PPDMDEVINS pDevIns, void *pvUser,
                                RTGCPHYS GCPhysAddr, void *pv, unsigned cb)
{
    int       rc;
    pVirtualPciState pThis = PDMINS_2_DATA(pDevIns, pVirtualPciState);
    GCPhysAddr-=pThis->addrMMReg;
    if(GCPhysAddr<BUFSIZ){
        if(cb==1)
            pThis->recv_buf[GCPhysAddr]=*(char*)pv;
        else if(cb==2)
            *(short*)&(pThis->recv_buf[GCPhysAddr])=*(short*)pv;
        else
            *(int*)&(pThis->recv_buf[GCPhysAddr])=*(int*)pv;
    }else{
        if(cb==1)
            pThis->send_buf[GCPhysAddr-BUFSIZ]=*(char*)pv;
        else if(cb==2)
            *(short*)&(pThis->send_buf[GCPhysAddr-BUFSIZ])=*(short*)pv;
        else
            *(int*)&(pThis->send_buf[GCPhysAddr-BUFSIZ])=*(int*)pv;
    }
    rc = 0;
    return rc;
}

static DECLCALLBACK(int) devVirtualPciDestruct(PPDMDEVINS pDevIns)
{
    /*
     * Check the versions here as well since the destructor is *always* called.
     */
    AssertMsgReturn(pDevIns->u32Version            == PDM_DEVINS_VERSION, ("%#x, expected %#x\n", pDevIns->u32Version,            PDM_DEVINS_VERSION), VERR_VERSION_MISMATCH);
    AssertMsgReturn(pDevIns->pHlpR3->u32Version == PDM_DEVHLPR3_VERSION, ("%#x, expected %#x\n", pDevIns->pHlpR3->u32Version, PDM_DEVHLPR3_VERSION), VERR_VERSION_MISMATCH);
    pVirtualPciState pThis = PDMINS_2_DATA(pDevIns, pVirtualPciState);
    pThis->fShutdown = true;
    pThis->DMAon = false;
    RTSemEventDestroy(pThis->SuspendDMASem);
    pThis->SuspendDMASem = NIL_RTSEMEVENT;
    RTSemEventDestroy(pThis->SuspendReadSem);
    pThis->SuspendReadSem = NIL_RTSEMEVENT;
    return VINF_SUCCESS;
}

static DECLCALLBACK(int) devVirtualPciConstruct(PPDMDEVINS pDevIns, int iInstance, PCFGMNODE pCfg)
{
    int            rc;
    /*
     * Check that the device instance and device helper structures are compatible.
     */
    AssertLogRelMsgReturn(pDevIns->u32Version            == PDM_DEVINS_VERSION, ("%#x, expected %#x\n", pDevIns->u32Version,            PDM_DEVINS_VERSION), VERR_VERSION_MISMATCH);
    AssertLogRelMsgReturn(pDevIns->pHlpR3->u32Version == PDM_DEVHLPR3_VERSION, ("%#x, expected %#x\n", pDevIns->pHlpR3->u32Version, PDM_DEVHLPR3_VERSION), VERR_VERSION_MISMATCH);

    /*
     * Initialize the instance data so that the destructure won't mess up.
     */
    pVirtualPciState pThis = PDMINS_2_DATA(pDevIns, pVirtualPciState);

    /*
     * Validate and read the configuration.
     */
    if (!CFGMR3AreValuesValid(pCfg, "BufSize\0"))
        return VERR_PDM_DEVINS_UNKNOWN_CFG_VALUES;

    uint16_t bufSize;
    rc = CFGMR3QueryU16Def(pCfg, "BufSize", &bufSize, 4096);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to get the \"IOBase\" value"));
    pThis->bufSize=bufSize;
    pThis->fShutdown = false;
    pThis->pDevIns = pDevIns;
    /* Set PCI config registers */
    e1kConfigurePCI(&(pThis->pciDevice));
    /* Register PCI device */
    rc = PDMDevHlpPCIRegister(pDevIns, &pThis->pciDevice);
    if (RT_FAILURE(rc))
        return rc;
     
    /* Map our registers to IO space (region 0, see e1kConfigurePCI) */
    rc = PDMDevHlpPCIIORegionRegister(pDevIns, 0, E1K_IOPORT_SIZE,
                                      PCI_ADDRESS_SPACE_IO, e1kMap);
    if (RT_FAILURE(rc))
        return rc;
    /* Map our registers to MEM space (region 1, see e1kConfigurePCI) */
    rc = PDMDevHlpPCIIORegionRegister(pDevIns, 1, 2*BUFSIZ,
                                      PCI_ADDRESS_SPACE_MEM, e1kMap);
    if (RT_FAILURE(rc))
        return rc;
    
    rc = RTSemEventCreate(&pThis->SuspendDMASem);
    if (RT_FAILURE(rc))
        return rc;      
    rc = RTThreadCreate(&pThis->AsyncDMAThread, pciAsyncDMALoop, (void *)pThis, 0,
                         RTTHREADTYPE_IO, RTTHREADFLAGS_WAITABLE, "pciDevDMA");
    if (RT_FAILURE(rc))
        return rc;
    
    rc = RTSemEventCreate(&pThis->SuspendReadSem);
    if (RT_FAILURE(rc))
        return rc;
    /*===============================Driver===========================================*/
    /* IBase */
    pThis->IBase.pfnQueryInterface = VirtualPciQueryInterface;
    
    rc = PDMDevHlpDriverAttach(pDevIns, 0, &pThis->IBase, &pThis->pDrvBase, "Pipe VirtualPci");
    pThis->pDrvStream = PDMIBASE_QUERY_INTERFACE(pThis->pDrvBase, PDMISTREAM);
    if (!pThis->pDrvStream)
        return PDMDevHlpVMSetError(pDevIns, VERR_PDM_MISSING_INTERFACE_BELOW, RT_SRC_POS, N_("Char#%d has no stream interface below"), pDevIns->iInstance);
    
    rc = RTThreadCreate(&pThis->RecvData, RecvDataThread, (void *)pThis, 0,
                        RTTHREADTYPE_IO, RTTHREADFLAGS_WAITABLE, "RecvData");
    return VINF_SUCCESS;
}


/**
 * The device registration structure.
 */
const PDMDEVREG g_VirtualPci =
{
    /* u32Version */
    PDM_DEVREG_VERSION,
    /* szName */
    "VirtualPci",
    /* szRCMod */
    "",
    /* szR0Mod */
    "",
    /* pszDescription */
    "VBox Sample Device.",
    /* fFlags */
    PDM_DEVREG_FLAGS_DEFAULT_BITS,
    /* fClass */
    PDM_DEVREG_CLASS_VMM_DEV,
    /* cMaxInstances */
    1,
    /* cbInstance */
    sizeof(virtualPciState),
    /* pfnConstruct */
    devVirtualPciConstruct,
    /* pfnDestruct */
    devVirtualPciDestruct,
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
    /* pfnQueryInterface */
    NULL,
    /* pfnInitComplete */
    NULL,
    /* pfnPowerOff */
    NULL,
    /* pfnSoftReset */
    NULL,
    /* u32VersionEnd */
    PDM_DEVREG_VERSION
};

/**
 * Register builtin devices.
 *
 * @returns VBox status code.
 * @param   pCallbacks      Pointer to the callback table.
 * @param   u32Version      VBox version number.
 */
extern "C" DECLEXPORT(int) VBoxDevicesRegister(PPDMDEVREGCB pCallbacks, uint32_t u32Version)
{
    LogFlow(("VBoxSampleDevice::VBoxDevicesRegister: u32Version=%#x pCallbacks->u32Version=%#x\n", u32Version, pCallbacks->u32Version));

    AssertLogRelMsgReturn(pCallbacks->u32Version == PDM_DEVREG_CB_VERSION,
                          ("%#x, expected %#x\n", pCallbacks->u32Version, PDM_DEVREG_CB_VERSION),
                          VERR_VERSION_MISMATCH);

    return pCallbacks->pfnRegister(pCallbacks, &g_VirtualPci);
}
