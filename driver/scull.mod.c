#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xa7672d5a, "module_layout" },
	{ 0xadf42bd5, "__request_region" },
	{ 0xba13492, "cdev_del" },
	{ 0x20623cac, "kmalloc_caches" },
	{ 0x53319a21, "pci_bus_read_config_byte" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0x74450b25, "cdev_init" },
	{ 0x6980fe91, "param_get_int" },
	{ 0xd8e484f0, "register_chrdev_region" },
	{ 0x105e2727, "__tracepoint_kmalloc" },
	{ 0x7485e15e, "unregister_chrdev_region" },
	{ 0xff964b25, "param_set_int" },
	{ 0x59d8223a, "ioport_resource" },
	{ 0xb72397d5, "printk" },
	{ 0x2f287f0d, "copy_to_user" },
	{ 0xb4390f9a, "mcount" },
	{ 0xfda85a7d, "request_threaded_irq" },
	{ 0xed24100d, "cdev_add" },
	{ 0x6eb25b1e, "kmem_cache_alloc" },
	{ 0x9bce482f, "__release_region" },
	{ 0x5a020d9d, "pci_unregister_driver" },
	{ 0x37a0cba, "kfree" },
	{ 0x29ecd539, "__pci_register_driver" },
	{ 0x8a802dbd, "pci_enable_device" },
	{ 0xd6c963c, "copy_from_user" },
	{ 0x29537c9e, "alloc_chrdev_region" },
	{ 0xf20dabd8, "free_irq" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("pci:v00001AF4d00001110sv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "7B7FA761D86B6CB8E823414");
