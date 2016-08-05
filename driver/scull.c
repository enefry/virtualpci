/*
 * complete.c -- the writers awake the readers
 *
 * Copyright (C) 2003 Alessandro Rubini and Jonathan Corbet
 * Copyright (C) 2003 O'Reilly & Associates
 * Copyright (C) 2010 Max Rozhkov <feruxmax@gmail.com>
 *
 * The source code in this file can be freely used, adapted,
 * and redistributed in source or binary form, so long as an
 * acknowledgment appears in derived source files.  The citation
 * should list that the code comes from the book "Linux Device
 * Drivers" by Alessandro Rubini and Jonathan Corbet, published
 * by O'Reilly & Associates.   No warranty is attached;
 * we cannot take responsibility for errors or fitness for use.
 *
 * $Id: complete.c,v 1.2 2004/09/26 07:02:43 gregkh Exp $
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

#include <linux/uaccess.h>	/* copy_*_user */

MODULE_LICENSE("GPL v2");

#define SCULL_MAJOR 0   /* dynamic major by default */
#define SCULL_NR_DEVS 1    /* scull0 through scull3 */
#define BUFSIZ 4096

#define MY_DEV_VENDOR_ID 0x1af4
#define MY_DEV_DEVICE_ID 0x1110

/* PCI device regs */
#define STATUS_REG                      0x0
#define COMAND_REG                      0x0
#define START_ADR_REG                   0x1
#define SIZE_REG                        0x2
#define DATA_REG                        0x3
/*PCI comands */
#define WRONG_CMD                       0x00000000
#define WRITE_CMD                       0x00000001 // write data from receiving buffer to memory
#define READ_CMD                        0x00000002 // read data from memory to sending boffer
#define SEND_CMD                        0x00000003 // send data from sending buffer
#define START_READ_CMD                  0x00000004 // start reading (sw on interrupts)
/*status */
#define DMA_READ_DONE                   0x00000001
#define DMA_WRITE_DONE                  0x00000002
#define DATA_RECVD                      0x00000003
#define BUFF_IS_EMPTY                   0x00000004
/*
 * Our parameters which can be set at load time.
 */

int scull_major =   SCULL_MAJOR;
int scull_minor =   0;
int scull_nr_devs = SCULL_NR_DEVS;	/* number of bare scull devices */
int irq = 0;
int dma=1;
int direct_io=0;

module_param(irq, int, S_IRUGO);
module_param(dma, int, S_IRUGO);
module_param(direct_io, int, S_IRUGO);

struct scull_dev {
    char *buf;
    int buf_bus_adr;
    unsigned long size;
    int recv_full;
    struct cdev cdev;	  /* Char device structure		*/
    /*pci hard */
    struct pci_dev *pciDev;
    u8 irq;
    unsigned long io_base;
    unsigned long io_size;
    unsigned long mem_base;
    unsigned long mem_size;
    unsigned long mem_region;
    unsigned long map_mem_region;
} *scull_devices;	/* allocated in scull_init_module */;


static struct pci_device_id ids[] = {
	{ PCI_DEVICE(MY_DEV_VENDOR_ID, MY_DEV_DEVICE_ID), },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, ids);

static unsigned char skel_get_revision(struct pci_dev *dev)
{
	u8 revision;

	pci_read_config_byte(dev, PCI_REVISION_ID, &revision);
	return revision;
}

static int probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	/* Do probing type stuff here.  
	 * Like calling request_region();
	 */
	pci_enable_device(dev);
	
	if (skel_get_revision(dev) == 0x42)
		return -ENODEV;
        scull_devices->pciDev = dev;
	return 0;
}

static void remove(struct pci_dev *dev)
{
	/* clean up any allocated resources and stuff here.
	 * like call release_region();
	 */
}

static struct pci_driver pci_driver = {
	.name = "pci_skel",
	.id_table = ids,
	.probe = probe,
	.remove = remove,
};


module_param(scull_major, int, S_IRUGO);
module_param(scull_minor, int, S_IRUGO);


/*
 * Open and close
 */

int scull_open(struct inode *inode, struct file *filp)
{
    struct scull_dev *dev; /* device information */
    dev = container_of(inode->i_cdev, struct scull_dev, cdev);
    filp->private_data = dev; /* for other methods */
    outl(START_READ_CMD, dev->io_base+COMAND_REG);
    
    printk(KERN_ALERT "Open\n");
    return 0;          /* success */
}

int scull_release(struct inode *inode, struct file *filp)
{
    struct scull_dev *dev; /* device information */
    dev = container_of(inode->i_cdev, struct scull_dev, cdev);
    printk(KERN_ALERT "Release\n");
     
    return 0;
}

/*
 * Data management: read and write
 */

ssize_t scull_read(struct file *filp, char __user *buf, size_t count,
                   loff_t *f_pos)
{
    struct scull_dev *dev = filp->private_data;
    char byte;
    int n_read=0;
    if(dma){
        if(dev->recv_full){
            count = (count > dev->size)? dev->size:count;
            printk(KERN_ALERT "Read using DMA %d\n",count);
            if (copy_to_user(buf, dev->buf, count)) {
                return -EFAULT;
            }
            dev->recv_full = 0;
        }
        else
        {
            count = 0;
        }
    }else{
        while(inl(dev->io_base+STATUS_REG)!=BUFF_IS_EMPTY && n_read<count){
            dev->buf[n_read++] = inb(dev->io_base+DATA_REG);
        }
        count = n_read<count?n_read:count; //если в буфере нет нужного количества байт
        printk(KERN_ALERT "Read using PIO %d\n",count);
        if (copy_to_user(buf, dev->buf, count)) {
            return -EFAULT;
        }
    }
    return count;
}

ssize_t scull_write(struct file *filp, const char __user *buf, size_t count,
                    loff_t *f_pos)
{
    int i;
    struct scull_dev *dev = filp->private_data;
    printk(KERN_ALERT "Write\n");
    count = (count > BUFSIZ)? BUFSIZ:count;
    if (copy_from_user(dev->buf, buf, count)) {
        return -EFAULT;
    }
    //    addr = dma_map_single(&dev->pciDev->dev, dev->buf, BUFSIZ, DMA_BIDIRECTIONAL);
    //
    //    dma_unmap_single(&dev->pciDev->dev, addr, BUFSIZ, DMA_BIDIRECTIONAL);
    
    if(dma){    
        outl(dev->buf_bus_adr, dev->io_base+START_ADR_REG);
        outl(count, dev->io_base+SIZE_REG);
        outl(READ_CMD, dev->io_base+COMAND_REG);
    }else if(direct_io){
        ;
    }else{
        for(i=0;i<count;i++){
            outb(dev->buf[i], dev->io_base + DATA_REG);
            printk(KERN_ALERT "PIO%d: %c\n",i ,dev->buf[i]);
        }
    }
    
    return count;
}

irqreturn_t scull_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
        struct scull_dev *dev = (struct scull_dev*)dev_id;
        
        unsigned int status = inl(dev->io_base+STATUS_REG);
        switch(status){
        case DMA_READ_DONE:
            iowrite8('!',dev->map_mem_region+4096+2);
            outl(SEND_CMD, dev->io_base+COMAND_REG);
            break;
        case DATA_RECVD:
            outl(dev->buf_bus_adr, dev->io_base+START_ADR_REG);  
            outl(WRITE_CMD, dev->io_base+COMAND_REG);
            break;
        case DMA_WRITE_DONE:
            dev->size = inl(dev->io_base+SIZE_REG);
            dev->recv_full = 1;
            break;
        default:
            printk(KERN_ALERT "Unknown!!! ");
        }
printk(KERN_ALERT "Irq! %u\n",status);
	return IRQ_HANDLED;
}

struct file_operations scull_fops = {
    .owner =    THIS_MODULE,
    .read =     scull_read,
    .write =    scull_write,
    .open =     scull_open,
    .release =  scull_release,
};


void scull_cleanup(void)
{
    dev_t devno = MKDEV(scull_major, scull_minor);
    cdev_del(&(scull_devices->cdev));
    /* cleanup_module is never called if registering failed */
    printk(KERN_ALERT "Cleanup %d\n",scull_devices->irq);
    free_irq(scull_devices->irq, scull_devices );
    unregister_chrdev_region(devno, scull_nr_devs);
    
    release_region(scull_devices->io_base, scull_devices->io_size);
    iounmap(scull_devices->map_mem_region);
    release_mem_region(scull_devices->mem_base,scull_devices->mem_size);
    pci_unregister_driver(&pci_driver);
    if(scull_devices->buf!=NULL)
        kfree(scull_devices->buf);
 
}

/*
 * Set up the char_dev structure for this device.
 */
static void scull_setup_cdev(struct scull_dev *dev)
{
	int err, devno = MKDEV(scull_major, scull_minor );
    
	cdev_init(&dev->cdev, &scull_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &scull_fops;
	err = cdev_add (&dev->cdev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_NOTICE "Error %d", err);
}

int scull_init(void)
{
    int result;
    dev_t dev = 0;
    /*
     * Get a range of minor numbers to work with, asking for a dynamic
     * major unless directed otherwise at load time.
     */
    if (scull_major) {
        dev = MKDEV(scull_major, scull_minor);
        result = register_chrdev_region(dev, scull_nr_devs, "scull");
    } else {
        result = alloc_chrdev_region(&dev, scull_minor, scull_nr_devs,
                                     "scull");
        scull_major = MAJOR(dev);
    }
    
    if (result < 0) {
        printk(KERN_WARNING "scull: can't get major %d\n", scull_major);
        return result;
    }
    
    scull_devices = kmalloc(scull_nr_devs * sizeof(struct scull_dev), GFP_KERNEL);
    if (!scull_devices) {
        result = -ENOMEM;
        goto fail;  /* Make this more graceful */
    }
    memset(scull_devices, 0, scull_nr_devs * sizeof(struct scull_dev));
    
    scull_setup_cdev(scull_devices);
    
    printk(KERN_ALERT "Succeed! Scull major is %d\n", scull_major);
    
    result = pci_register_driver(&pci_driver);
    
    scull_devices->buf=NULL;
    scull_devices->buf = kmalloc(BUFSIZ, GFP_KERNEL);
    
    /* IO Res */
    scull_devices->io_base = pci_resource_start(scull_devices->pciDev, 0);
    scull_devices->io_size = pci_resource_end(scull_devices->pciDev, 0) - scull_devices->io_base;
    
    if(request_region(scull_devices->io_base, scull_devices->io_size, "Scull")==NULL){
        printk(KERN_ALERT "request IO region error\n");
        goto fail;
    }
    scull_devices->buf_bus_adr=virt_to_bus(scull_devices->buf);
    
    /*MEM Res*/
    scull_devices->mem_base = pci_resource_start(scull_devices->pciDev, 1);
    scull_devices->mem_size = pci_resource_end(scull_devices->pciDev, 1) - scull_devices->mem_base;
    
    scull_devices->mem_region = request_mem_region(scull_devices->mem_base,
                                                 scull_devices->mem_size,
                                                 "Scull");
    if(!scull_devices->mem_region){
        printk(KERN_ALERT "request IO region error\n");
        goto fail;
    }
    scull_devices->map_mem_region = ioremap(scull_devices->mem_base, scull_devices->mem_size);
    
    printk(KERN_ALERT "request Mem region. mem_base: 0x%x mem_region: 0x%x 0x%x\n",
           scull_devices->mem_base, scull_devices->mem_region, scull_devices->map_mem_region);

    /* IRQ */
    if(dma){
        result = pci_read_config_byte(scull_devices->pciDev, PCI_INTERRUPT_LINE, &scull_devices->irq);
        printk(KERN_ALERT "Irq: %d\n",scull_devices->irq);
        if(irq!=0)
            scull_devices->irq = irq;
        result = request_irq(scull_devices->irq, scull_interrupt,0 , "scull", scull_devices);
        scull_devices->recv_full = 0;
        if (result) {
            goto fail;
        }
    }
    return result; /* succeed */
    
    fail:
    scull_cleanup();
    return result;
}

module_init(scull_init);
module_exit(scull_cleanup);
