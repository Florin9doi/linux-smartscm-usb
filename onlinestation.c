#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#define ONLINESTATION_VENDOR_ID   0x05db
#define ONLINESTATION_PRODUCT_ID  0x0006

#define ONLINESTATION_EP_DATA_IN   0x81
#define ONLINESTATION_EP_DATA_OUT  0x02
#define ONLINESTATION_EP_INT_IN    0x83

#define ONLINESTATION_BUF_SIZE     4096
#define ONLINESTATION_INT_BUF_SIZE 8

#define ONLINESTATION_REQ_SET_BAUD       0x10
#define ONLINESTATION_REQ_SET_LINE_STATE 0x11
#define ONLINESTATION_REQ_SET_DATA_BITS  0x12
#define ONLINESTATION_REQ_ENABLE         0xd0

static const u32 onlinestation_rates[] = {
    110, 300, 600, 1200, 2400, 4800, 9600,
    19200, 38400, 57600, 115200, 230400, 460800, 921600
};

struct onlinestation_port {
    struct usb_serial_port *port;

    struct urb *read_urb;
    u8 *read_buf;

    struct urb *write_urb;
    u8 *write_buf;
    bool write_busy;
    spinlock_t lock;

    struct urb *int_urb;
    u8 *int_buf;

    u8 mcr;
    u8 msr;
};

static int onlinestation_vendor_cmd(struct usb_serial_port *port,
                                    u8 request, u16 value)
{
    struct usb_device *udev = port->serial->dev;

    dev_info(&port->dev,
        "OS: vendor cmd req=0x%02x wValue=0x%04x\n",
        request, value);

    return usb_control_msg(udev,
        usb_sndctrlpipe(udev, 0),
        request,
        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
        value,
        0,
        NULL,
        0,
        1000);
}

static void onlinestation_read_bulk_callback(struct urb *urb)
{
    struct onlinestation_port *sp = urb->context;
    struct usb_serial_port *port = sp->port;
    struct tty_struct *tty;
    int status = urb->status;

    if (!port || !port->serial)
        return;

    if (status == -ESHUTDOWN || status == -ENOENT || status == -ECONNRESET)
        return;

    if (status) {
        dev_warn(&port->dev, "RX urb error: %d\n", status);
        return;
    }

    tty = tty_port_tty_get(&port->port);
    if (tty) {
        u8 *buf = urb->transfer_buffer;
        int len = urb->actual_length;

        if (len > 0) {
            tty_insert_flip_string(&port->port, buf, len);
            tty_flip_buffer_push(&port->port);
        }

        tty_kref_put(tty);
    }
}

static void onlinestation_int_callback(struct urb *urb)
{
    struct onlinestation_port *sp = urb->context;
    struct usb_serial_port *port = sp->port;
    struct usb_device *udev;
    int status = urb->status;

    if (!port || !port->serial)
        return;

    udev = port->serial->dev;

    if (status == -ESHUTDOWN || status == -ENOENT || status == -ECONNRESET)
        return;

    if (status) {
        dev_warn(&port->dev, "INT urb error: %d\n", status);
        goto resubmit;
    }

    if (urb->actual_length >= 2 && (sp->int_buf[0] != 0x04 || sp->int_buf[1] != 0x01))
        dev_info(&port->dev, "OS: int_callback 0x%02x/0x%02x\n", sp->int_buf[0], sp->int_buf[1]);

    if (urb->actual_length >= 1 && (sp->int_buf[0] & 0x01)) {

        usb_fill_bulk_urb(sp->read_urb, udev,
                  usb_rcvbulkpipe(udev, ONLINESTATION_EP_DATA_IN),
                  sp->read_buf, ONLINESTATION_BUF_SIZE,
                  onlinestation_read_bulk_callback, sp);

        if (usb_submit_urb(sp->read_urb, GFP_ATOMIC))
            dev_err(&port->dev, "submit RX urb failed\n");
    }
    if (urb->actual_length >= 2)
        sp->msr = sp->int_buf[1];

resubmit:
    if (usb_submit_urb(urb, GFP_ATOMIC))
        dev_err(&port->dev, "INT urb resubmit failed\n");
}

static void onlinestation_write_bulk_callback(struct urb *urb)
{
    struct onlinestation_port *sp = urb->context;
    unsigned long flags;

    spin_lock_irqsave(&sp->lock, flags);
    sp->write_busy = false;
    spin_unlock_irqrestore(&sp->lock, flags);

    if (urb->status)
        dev_warn(&sp->port->dev, "TX urb error: %d\n", urb->status);

    tty_port_tty_wakeup(&sp->port->port);
}

static int onlinestation_write(struct tty_struct *tty,
                          struct usb_serial_port *port,
                          const unsigned char *buf, int count)
{
    struct onlinestation_port *sp = usb_get_serial_port_data(port);
    unsigned long flags;
    int ret;

    if (!sp)
        return -ENODEV;

    if (!count)
        return 0;

    spin_lock_irqsave(&sp->lock, flags);

    if (sp->write_busy) {
        spin_unlock_irqrestore(&sp->lock, flags);
        return 0;
    }

    count = min_t(int, count, ONLINESTATION_BUF_SIZE);

    memcpy(sp->write_buf, buf, count);
    sp->write_busy = true;

    spin_unlock_irqrestore(&sp->lock, flags);

    usb_fill_bulk_urb(sp->write_urb,
        port->serial->dev,
        usb_sndbulkpipe(port->serial->dev, ONLINESTATION_EP_DATA_OUT),
        sp->write_buf,
        count,
        onlinestation_write_bulk_callback,
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

static unsigned int onlinestation_write_room(struct tty_struct *tty)
{
    struct usb_serial_port *port = tty->driver_data;
    struct onlinestation_port *sp = usb_get_serial_port_data(port);
    unsigned long flags;
    unsigned int room;

    if (!sp)
        return 0;

    spin_lock_irqsave(&sp->lock, flags);
    room = sp->write_busy ? 0 : ONLINESTATION_BUF_SIZE;
    spin_unlock_irqrestore(&sp->lock, flags);

    return room;
}

static u16 onlinestation_baud_index(unsigned int baud)
{
    int i;
    u32 best_diff = UINT_MAX;
    u16 best_idx = 0;

    for (i = 0; i < ARRAY_SIZE(onlinestation_rates); i++) {
        u32 r = onlinestation_rates[i];
        u32 diff = (r > baud) ? (r - baud) : (baud - r);

        if (diff < best_diff) {
            best_diff = diff;
            best_idx = i;
        }
    }

    return best_idx;
}

static int onlinestation_open(struct tty_struct *tty,
                         struct usb_serial_port *port)
{
    struct onlinestation_port *sp = usb_get_serial_port_data(port);
    struct usb_device *udev = port->serial->dev;
    int ret;

    if (!sp)
        return -ENODEV;

    ret = usb_set_interface(udev, 0, 0);
    if (ret)
        dev_warn(&port->dev, "usb_set_interface failed: %d\n", ret);

    onlinestation_vendor_cmd(port, ONLINESTATION_REQ_ENABLE, 0);
    onlinestation_vendor_cmd(port, ONLINESTATION_REQ_SET_BAUD, onlinestation_baud_index(115200));

    sp->mcr = UART_MCR_RTS | UART_MCR_DTR;
    onlinestation_vendor_cmd(port, ONLINESTATION_REQ_SET_LINE_STATE, sp->mcr);

    usb_fill_int_urb(sp->int_urb, udev,
        usb_rcvintpipe(udev, ONLINESTATION_EP_INT_IN),
        sp->int_buf, ONLINESTATION_INT_BUF_SIZE,
        onlinestation_int_callback, sp,
        1);

    ret = usb_submit_urb(sp->int_urb, GFP_KERNEL);
    if (ret)
        dev_err(&port->dev, "submit INT urb failed: %d\n", ret);

    return ret;
}

static void onlinestation_close(struct usb_serial_port *port)
{
    struct onlinestation_port *sp = usb_get_serial_port_data(port);

    if (!sp)
        return;

    usb_kill_urb(sp->read_urb);
    usb_kill_urb(sp->write_urb);
    usb_kill_urb(sp->int_urb);

    sp->mcr = 0;
    onlinestation_vendor_cmd(port, ONLINESTATION_REQ_SET_LINE_STATE, sp->mcr);
}

static int onlinestation_port_probe(struct usb_serial_port *port)
{
    struct onlinestation_port *sp;

    sp = kzalloc(sizeof(*sp), GFP_KERNEL);
    if (!sp)
        return -ENOMEM;

    sp->port = port;

    sp->read_buf = kmalloc(ONLINESTATION_BUF_SIZE, GFP_KERNEL);
    sp->write_buf = kmalloc(ONLINESTATION_BUF_SIZE, GFP_KERNEL);
    sp->int_buf = kmalloc(ONLINESTATION_INT_BUF_SIZE, GFP_KERNEL);

    if (!sp->read_buf || !sp->write_buf || !sp->int_buf)
        goto err;

    sp->read_urb = usb_alloc_urb(0, GFP_KERNEL);
    sp->write_urb = usb_alloc_urb(0, GFP_KERNEL);
    sp->int_urb = usb_alloc_urb(0, GFP_KERNEL);

    if (!sp->read_urb || !sp->write_urb || !sp->int_urb)
        goto err;

    spin_lock_init(&sp->lock);

    usb_set_serial_port_data(port, sp);

    return 0;

err:
    usb_free_urb(sp->read_urb);
    usb_free_urb(sp->write_urb);
    usb_free_urb(sp->int_urb);
    kfree(sp->read_buf);
    kfree(sp->write_buf);
    kfree(sp->int_buf);
    kfree(sp);
    return -ENOMEM;
}

static void onlinestation_port_remove(struct usb_serial_port *port)
{
    struct onlinestation_port *sp = usb_get_serial_port_data(port);

    if (!sp)
        return;

    usb_kill_urb(sp->read_urb);
    usb_kill_urb(sp->write_urb);
    usb_kill_urb(sp->int_urb);

    usb_free_urb(sp->read_urb);
    usb_free_urb(sp->write_urb);
    usb_free_urb(sp->int_urb);
    kfree(sp->read_buf);
    kfree(sp->write_buf);
    kfree(sp->int_buf);
    usb_set_serial_port_data(port, NULL);
    kfree(sp);
}

static void onlinestation_set_termios(struct tty_struct *tty,
                                 struct usb_serial_port *port,
                                 const struct ktermios *old)
{
    struct onlinestation_port *sp = usb_get_serial_port_data(port);
    unsigned int baud;

    if (!sp)
        return;

    baud = tty_get_baud_rate(tty);
    if (!baud)
        baud = 115200;

    onlinestation_vendor_cmd(port, ONLINESTATION_REQ_SET_BAUD, onlinestation_baud_index(baud));

    // switch (tty->termios.c_cflag & CSIZE) {
    //     case CS5: lcr |= UART_LCR_WLEN5; break;
    //     case CS6: lcr |= UART_LCR_WLEN6; break;
    //     case CS7: lcr |= UART_LCR_WLEN7; break;
    //     default:
    //     case CS8: lcr |= UART_LCR_WLEN8; break;
    // }

    // if (tty->termios.c_cflag & CSTOPB)
    //     lcr |= UART_LCR_STOP;

    // if (tty->termios.c_cflag & PARENB) {
    //     lcr |= UART_LCR_PARITY;
    //     if (!(tty->termios.c_cflag & PARODD))
    //         lcr |= UART_LCR_EPAR;
    // }

    // onlinestation_vendor_cmd(port, ONLINESTATION_REQ_SET_DATA_BITS, lcr);

    if (tty->termios.c_cflag & CBAUD)
        sp->mcr |= UART_MCR_DTR;

    if (tty->termios.c_cflag & CRTSCTS)
        sp->mcr |= UART_MCR_RTS;

    onlinestation_vendor_cmd(port, ONLINESTATION_REQ_SET_LINE_STATE, sp->mcr);
}

static int onlinestation_tiocmset(struct tty_struct *tty,
                             unsigned int set,
                             unsigned int clear)
{
    struct usb_serial_port *port = tty->driver_data;
    struct onlinestation_port *sp = usb_get_serial_port_data(port);
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

    return onlinestation_vendor_cmd(port, ONLINESTATION_REQ_SET_LINE_STATE, sp->mcr);
}

static int onlinestation_tiocmget(struct tty_struct *tty)
{
    struct usb_serial_port *port = tty->driver_data;
    struct onlinestation_port *sp = usb_get_serial_port_data(port);
    unsigned int result = 0;

    if (!sp)
        return -ENODEV;

    // out
    if (sp->mcr & UART_MCR_RTS)
        result |= TIOCM_RTS;
    if (sp->mcr & UART_MCR_DTR)
        result |= TIOCM_DTR;

    // in
    if (sp->msr & 0x01)
        result |= TIOCM_CTS;
    if (sp->msr & 0x02)
        result |= TIOCM_DSR;
    if (sp->msr & 0x04)
        result |= TIOCM_RI;
    if (sp->msr & 0x08) // UART_MSR_DCD
        result |= TIOCM_CD;

    return result;
}

static const struct usb_device_id onlinestation_id_table[] = {
    { USB_DEVICE(ONLINESTATION_VENDOR_ID, ONLINESTATION_PRODUCT_ID) },
    {}
};
MODULE_DEVICE_TABLE(usb, onlinestation_id_table);

static struct usb_serial_driver onlinestation_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name  = "onlinestation",
    },
    .id_table    = onlinestation_id_table,
    .num_ports   = 1,
    .port_probe  = onlinestation_port_probe,
    .port_remove = onlinestation_port_remove,
    .open        = onlinestation_open,
    .close       = onlinestation_close,
    .write       = onlinestation_write,
    .write_room  = onlinestation_write_room,
    .set_termios = onlinestation_set_termios,
    .tiocmset    = onlinestation_tiocmset,
    .tiocmget    = onlinestation_tiocmget,
};

static struct usb_serial_driver * const onlinestation_drivers[] = {
    &onlinestation_driver, NULL
};

module_usb_serial_driver(onlinestation_drivers, onlinestation_id_table);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Florin9doi");
MODULE_DESCRIPTION("OnlineStation Modem Driver");
