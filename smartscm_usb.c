#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#define SMARTSCM_VENDOR_ID   0x0572
#define SMARTSCM_PRODUCT_ID  0x1272

#define SMARTSCM_EP_CFG_OUT   0x01
#define SMARTSCM_EP_DATA_OUT  0x03
#define SMARTSCM_EP_DATA_IN   0x83
#define SMARTSCM_EP_GPIO_OUT  0x04

#define SMARTSCM_BUF_SIZE     4096

// ep1
#define SMARTSCM_SET_REG      0x40

// ep4
#define SMARTSCM_DEV_OFF      0xC9
#define SMARTSCM_DEV_ON       0xC1
#define SMARTSCM_LED_OFF      0x00
#define SMARTSCM_LED_READY    0x01
#define SMARTSCM_LED_CARRIER  0x02

struct smartscm_port {
    struct usb_serial_port *port;

    struct urb *read_urb;
    u8 *read_buf;

    struct urb *write_urb;
    u8 *write_buf;
    bool write_busy;
    spinlock_t lock;

    u8 mcr;
    u8 msr;

    struct work_struct dcd_work;
    u8 dcd_val;
};

static int smartscm_bulk_send(struct usb_serial_port *port,
                              unsigned int pipe,
                              const void *buf, size_t len,
                              int timeout)
{
    struct usb_device *udev = port->serial->dev;
    int actual, ret;

    ret = usb_bulk_msg(udev, pipe, (void *)buf, len, &actual, timeout);
    if (ret) {
        dev_err(&port->dev, "bulk send failed: %d\n", ret);
        return ret;
    }

    if (actual != len)
        dev_warn(&port->dev, "short write (%d/%zu)\n", actual, len);

    return 0;
}

static int smartscm_set_register(struct usb_serial_port *port,
                                 u8 reg, u8 value)
{
    u8 pkt[3] = { SMARTSCM_SET_REG, reg, value };

    return smartscm_bulk_send(port,
        usb_sndbulkpipe(port->serial->dev, SMARTSCM_EP_CFG_OUT),
        pkt, sizeof(pkt), 1000);
}

static int smartscm_gpio_control(struct usb_serial_port *port,
                                 u8 msb, u8 lsb)
{
    u8 pkt[4] = {0x00, 0x00, msb, lsb};

    return smartscm_bulk_send(port,
        usb_sndbulkpipe(port->serial->dev, SMARTSCM_EP_GPIO_OUT),
        pkt, sizeof(pkt), 1000);
}

static void smartscm_dcd_work(struct work_struct *work)
{
    struct smartscm_port *sp =
        container_of(work, struct smartscm_port, dcd_work);

    u8 leds = SMARTSCM_LED_READY;
    if (sp->dcd_val)
        leds |= SMARTSCM_LED_CARRIER;

    smartscm_gpio_control(sp->port, SMARTSCM_DEV_ON, leds);
}

static void smartscm_read_bulk_callback(struct urb *urb)
{
    struct smartscm_port *sp = urb->context;
    struct usb_serial_port *port = sp->port;
    struct tty_struct *tty;
    int status = urb->status;

    if (!port || !port->serial)
        return;

    if (status == -ESHUTDOWN || status == -ENOENT || status == -ECONNRESET)
        return;

    if (status) {
        dev_warn(&port->dev, "urb error: %d\n", status);
        goto resubmit;
    }

    tty = tty_port_tty_get(&port->port);
    if (tty) {
        u8 *buf = urb->transfer_buffer;
        int len = urb->actual_length;
        int i;

        if (len >= 1) {
            bool last_dcd = !!(sp->msr & UART_MSR_DCD);
            bool dcd = !!(buf[0] & UART_MSR_DCD);
            if (dcd != last_dcd) {
                usb_serial_handle_dcd_change(port, tty, dcd);
                sp->dcd_val = dcd;
                schedule_work(&sp->dcd_work);
            }

            sp->msr = buf[0];
        }

        if (len > 2) {
            for (i = 2; i < len; i += 2) {
                u8 data = buf[i];
                tty_insert_flip_char(&port->port, data, TTY_NORMAL);
            }

            tty_flip_buffer_push(&port->port);
        }

        tty_kref_put(tty);
    }

resubmit:
    if (usb_submit_urb(urb, GFP_ATOMIC))
        dev_err(&port->dev, "urb resubmit failed\n");
}

static void smartscm_write_bulk_callback(struct urb *urb)
{
    struct smartscm_port *sp = urb->context;
    unsigned long flags;

    spin_lock_irqsave(&sp->lock, flags);
    sp->write_busy = false;
    spin_unlock_irqrestore(&sp->lock, flags);

    if (urb->status)
        dev_warn(&sp->port->dev, "TX urb error: %d\n", urb->status);

    tty_port_tty_wakeup(&sp->port->port);
}

static int smartscm_write(struct tty_struct *tty,
                          struct usb_serial_port *port,
                          const unsigned char *buf, int count)
{
    struct smartscm_port *sp = usb_get_serial_port_data(port);
    unsigned long flags;
    int ret;

    if (!count)
        return 0;

    spin_lock_irqsave(&sp->lock, flags);

    if (sp->write_busy) {
        spin_unlock_irqrestore(&sp->lock, flags);
        return 0;
    }

    count = min_t(int, count, SMARTSCM_BUF_SIZE);

    memcpy(sp->write_buf, buf, count);
    sp->write_busy = true;

    spin_unlock_irqrestore(&sp->lock, flags);

    usb_fill_bulk_urb(sp->write_urb,
        port->serial->dev,
        usb_sndbulkpipe(port->serial->dev, SMARTSCM_EP_DATA_OUT),
        sp->write_buf,
        count,
        smartscm_write_bulk_callback,
        sp);

    ret = usb_submit_urb(sp->write_urb, GFP_ATOMIC);
    if (ret) {
        dev_err(&port->dev, "submit TX urb failed: %d\n", ret);

        spin_lock_irqsave(&sp->lock, flags);
        sp->write_busy = false;
        spin_unlock_irqrestore(&sp->lock, flags);

        return ret;
    }

    return count;
}

static unsigned int smartscm_write_room(struct tty_struct *tty)
{
    struct usb_serial_port *port = tty->driver_data;
    struct smartscm_port *sp = usb_get_serial_port_data(port);
    unsigned long flags;
    unsigned int room;

    spin_lock_irqsave(&sp->lock, flags);
    room = sp->write_busy ? 0 : SMARTSCM_BUF_SIZE;
    spin_unlock_irqrestore(&sp->lock, flags);

    return room;
}

static int smartscm_open(struct tty_struct *tty,
                         struct usb_serial_port *port)
{
    struct smartscm_port *sp = usb_get_serial_port_data(port);
    struct usb_device *udev = port->serial->dev;
    int ret;

    if (!sp)
        return -ENODEV;

    ret = usb_set_interface(udev, 0, 0);
    if (ret)
        dev_warn(&port->dev, "usb_set_interface failed: %d\n", ret);

    smartscm_gpio_control(port, SMARTSCM_DEV_OFF, SMARTSCM_LED_OFF);
    smartscm_gpio_control(port, SMARTSCM_DEV_ON, SMARTSCM_LED_OFF);

    smartscm_set_register(port, UART_LCR, UART_LCR_DLAB);
    smartscm_set_register(port, UART_DLL, 0x02);
    smartscm_set_register(port, UART_DLM, 0x00);
    smartscm_set_register(port, UART_FCR, UART_FCR_ENABLE_FIFO); // 0xE1
    smartscm_set_register(port, UART_LCR, UART_LCR_WLEN8);

    sp->mcr = UART_MCR_RTS | UART_MCR_DTR;
    smartscm_set_register(port, UART_MCR, sp->mcr);

    usb_fill_bulk_urb(sp->read_urb, udev,
        usb_rcvbulkpipe(udev, SMARTSCM_EP_DATA_IN),
        sp->read_buf, SMARTSCM_BUF_SIZE,
        smartscm_read_bulk_callback, sp);

    ret = usb_submit_urb(sp->read_urb, GFP_KERNEL);
    if (ret)
        dev_err(&port->dev, "submit RX urb failed: %d\n", ret);

    return ret;
}

static void smartscm_close(struct usb_serial_port *port)
{
    struct smartscm_port *sp = usb_get_serial_port_data(port);

    if (!sp)
        return;

    cancel_work_sync(&sp->dcd_work);
    usb_kill_urb(sp->read_urb);
    usb_kill_urb(sp->write_urb);

    smartscm_set_register(port, UART_MCR, 0x00);
    smartscm_set_register(port, UART_FCR, UART_FCR_ENABLE_FIFO
                                        | UART_FCR_CLEAR_RCVR);
    smartscm_set_register(port, UART_MCR, UART_MCR_RTS);

    smartscm_gpio_control(port, SMARTSCM_DEV_ON, SMARTSCM_LED_OFF);
}

static int smartscm_port_probe(struct usb_serial_port *port)
{
    struct smartscm_port *sp;

    sp = kzalloc(sizeof(*sp), GFP_KERNEL);
    if (!sp)
        return -ENOMEM;

    sp->port = port;

    sp->read_buf = kmalloc(SMARTSCM_BUF_SIZE, GFP_KERNEL);
    sp->write_buf = kmalloc(SMARTSCM_BUF_SIZE, GFP_KERNEL);

    if (!sp->read_buf || !sp->write_buf)
        goto err;

    sp->read_urb = usb_alloc_urb(0, GFP_KERNEL);
    sp->write_urb = usb_alloc_urb(0, GFP_KERNEL);

    if (!sp->read_urb || !sp->write_urb)
        goto err;

    spin_lock_init(&sp->lock);

    INIT_WORK(&sp->dcd_work, smartscm_dcd_work);

    usb_set_serial_port_data(port, sp);

    return 0;

err:
    usb_free_urb(sp->read_urb);
    usb_free_urb(sp->write_urb);
    kfree(sp->read_buf);
    kfree(sp->write_buf);
    kfree(sp);
    return -ENOMEM;
}

static void smartscm_port_remove(struct usb_serial_port *port)
{
    struct smartscm_port *sp = usb_get_serial_port_data(port);

    if (!sp)
        return;

    cancel_work_sync(&sp->dcd_work);
    usb_kill_urb(sp->read_urb);
    usb_kill_urb(sp->write_urb);

    usb_free_urb(sp->read_urb);
    usb_free_urb(sp->write_urb);
    kfree(sp->read_buf);
    kfree(sp->write_buf);
    usb_set_serial_port_data(port, NULL);
    kfree(sp);
}

static void smartscm_set_termios(struct tty_struct *tty,
                                 struct usb_serial_port *port,
                                 const struct ktermios *old)
{
    unsigned int baud, divisor;
    u8 lcr = 0;
    u8 mcr = 0;

    baud = tty_get_baud_rate(tty);
    if (!baud)
        baud = 57600;

    divisor = 1843200 / (16 * baud);

    smartscm_set_register(port, UART_LCR, UART_LCR_DLAB);
    smartscm_set_register(port, UART_DLL, divisor & 0xFF);
    smartscm_set_register(port, UART_DLM, (divisor >> 8) & 0xFF);

    switch (tty->termios.c_cflag & CSIZE) {
        case CS5: lcr |= UART_LCR_WLEN5; break;
        case CS6: lcr |= UART_LCR_WLEN6; break;
        case CS7: lcr |= UART_LCR_WLEN7; break;
        default:
        case CS8: lcr |= UART_LCR_WLEN8; break;
    }

    if (tty->termios.c_cflag & CSTOPB)
        lcr |= UART_LCR_STOP;

    if (tty->termios.c_cflag & PARENB) {
        lcr |= UART_LCR_PARITY;
        if (!(tty->termios.c_cflag & PARODD))
            lcr |= UART_LCR_EPAR;
    }

    smartscm_set_register(port, UART_LCR, lcr);

    if (tty->termios.c_cflag & CBAUD)
        mcr |= UART_MCR_DTR;

    if (tty->termios.c_cflag & CRTSCTS)
        mcr |= UART_MCR_RTS;

    smartscm_set_register(port, UART_MCR, mcr);
}

static int smartscm_tiocmset(struct tty_struct *tty,
                             unsigned int set,
                             unsigned int clear)
{
    struct usb_serial_port *port = tty->driver_data;
    struct smartscm_port *sp = usb_get_serial_port_data(port);
    unsigned long flags;

    if (!sp)
        return -ENODEV;

    spin_lock_irqsave(&sp->lock, flags);

    if (set & TIOCM_RTS)
        sp->mcr |= UART_MCR_RTS;
    if (set & TIOCM_DTR)
        sp->mcr |= UART_MCR_DTR;

    if (clear & TIOCM_RTS)
        sp->mcr &= ~UART_MCR_RTS;
    if (clear & TIOCM_DTR)
        sp->mcr &= ~UART_MCR_DTR;

    spin_unlock_irqrestore(&sp->lock, flags);

    return smartscm_set_register(port, UART_MCR, sp->mcr);
}

static int smartscm_tiocmget(struct tty_struct *tty)
{
    struct usb_serial_port *port = tty->driver_data;
    struct smartscm_port *sp = usb_get_serial_port_data(port);
    unsigned int result = 0;

    if (!sp)
        return -ENODEV;

    // out
    if (sp->mcr & UART_MCR_RTS)
        result |= TIOCM_RTS;
    if (sp->mcr & UART_MCR_DTR)
        result |= TIOCM_DTR;

    // in
    if (sp->msr & UART_MSR_CTS)
        result |= TIOCM_CTS;
    if (sp->msr & UART_MSR_DSR)
        result |= TIOCM_DSR;
    if (sp->msr & UART_MSR_RI)
        result |= TIOCM_RI;
    if (sp->msr & UART_MSR_DCD)
        result |= TIOCM_CD;

    return result;
}

static const struct usb_device_id smartscm_id_table[] = {
    { USB_DEVICE(SMARTSCM_VENDOR_ID, SMARTSCM_PRODUCT_ID) },
    {}
};
MODULE_DEVICE_TABLE(usb, smartscm_id_table);

static struct usb_serial_driver smartscm_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name  = "smartscm",
    },
    .id_table    = smartscm_id_table,
    .num_ports   = 1,
    .port_probe  = smartscm_port_probe,
    .port_remove = smartscm_port_remove,
    .open        = smartscm_open,
    .close       = smartscm_close,
    .write       = smartscm_write,
    .write_room  = smartscm_write_room,
    .set_termios = smartscm_set_termios,
    .tiocmset    = smartscm_tiocmset,
    .tiocmget    = smartscm_tiocmget,
};

static struct usb_serial_driver * const smartscm_drivers[] = {
    &smartscm_driver, NULL
};

module_usb_serial_driver(smartscm_drivers, smartscm_id_table);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Florin9doi");
MODULE_DESCRIPTION("SmartSCM-USB Modem Driver");
