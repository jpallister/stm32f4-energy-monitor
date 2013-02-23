#include <stdio.h>
#include <libusb-1.0/libusb.h>
#include <time.h>
#include <signal.h>

libusb_device_handle *devh = NULL;
libusb_device *dev = NULL;

void ctrlc(int signum)
{
    char buf[] = "F";
    int len, r;

    printf("Closing...\n");

    if(devh)
    {
        libusb_bulk_transfer(devh, 0x1, buf, 1, &len, 0);

        r = 0;
        while(len > 0 && r >= 0)
            r = libusb_bulk_transfer(devh, 0x81, buf, 1, &len, 100);
        libusb_release_interface(devh,0 );
        libusb_close(devh);
    }
    if(dev)
    {
    }
    exit(0);
}

void access_device(libusb_device *dev)
{
    int r, len=0;
    int t1, t2, i, tot, c=0;
    char buff[1];
    FILE *out = fopen("out_results","w");

    r = libusb_open(dev, &devh);
    if(r < 0)
    {
        printf("Error opening device %d\n", r);
        return;
    }

    r = libusb_claim_interface(devh, 0);
    if(r < 0)
    {
        printf("Error claiming interface %d\n", r);
        return;
    }


    printf("Sending start\n");
    buff[0] = 'S';
    libusb_bulk_transfer(devh, 0x1, buff, 1, &len, 0);
    printf("Transferring\n");
    for(;;)
    {
        unsigned char buf[2048*2]={0};
        int cur_rate = 0, cur_time = 0;
        short b1, b2;
        int samp=0;

        t1 = time(0);
        tot = 0;
  //      printf("Starting measurement\n");

        while(time(0) < t1 + 2)
        {
            libusb_bulk_transfer(devh, 0x81, buf, 2048*2, &len, 0);
            tot += len;
     //       continue;
            //for(i = 0; i < len/2;++i)
                //printf("%c%c", buf[i]&0xff, buf[i]>>8);
               // printf("%d\n", buf[i]);
            for(i = 0; i < len; ++i, c = (c + 1) % 64)
                if(c == 0)
                {
                    cur_rate = buf[i];
     //               printf("%d\n", cur_rate);
                }
                else
                {
                    if((c-1)%3 == 0)
                    {
                        b1 = buf[i];
                    }
                    else if((c-1)%3 == 1)
                    {
                        b2 = buf[i];
                    }
                    else
                    {
                        b1 |= (buf[i]&0x0F)<<8;
                        b2 |= (buf[i]&0xF0)<<4;
                        fprintf(out, "%d %d\n",b1,cur_time);
                        cur_time += cur_rate;
                        fprintf(out, "%d %d\n",b2,cur_time);
                        cur_time += cur_rate;
                        samp += 2;
                    }
                }


        }
        printf("\t===> %f bytes/s (%f samples/s)\n", tot/2., samp/2.);
    }


    libusb_release_interface(devh, 0);
}

int main()
{
    libusb_device **devs;
    int r, i=0;
    ssize_t cnt;

    r = libusb_init(NULL);
    if (r < 0)
        return r;

    cnt = libusb_get_device_list(NULL, &devs);
    if (cnt < 0)
          return (int) cnt;

    signal(SIGINT, ctrlc);

    while ((dev = devs[i++]) != NULL) {
        struct libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(dev, &desc);
        if (r < 0) {
                fprintf(stderr, "failed to get device descriptor");
                return;
        }

        printf("%04x:%04x (bus %d, device %d)\n",
                desc.idVendor, desc.idProduct,
                libusb_get_bus_number(dev), libusb_get_device_address(dev));
        if(desc.idVendor == 0x1337 && desc.idProduct == 0x1337)
            access_device(dev);
    }

    libusb_free_device_list(devs, 1);

    libusb_exit(NULL);

    return 0;
}
