char ReportDescriptor[34] =
{ 　　　　　　　　　　　　　　　　　　　　//这里34就是前面建立工程第五点说的，报文描述符大小
    0x06, 0x00, 0xff,              // USAGE_PAGE (Vendor Defined Page 1) 表示一个报文标签之类的用途类页
    0x09, 0x01,                    // USAGE (Vendor Usage 1) 表示一个报告ID标志
    0xa1, 0x01,                    // COLLECTION (Application) 表示应用集合，要以下面最后的0xc0结束它


    0x09, 0x01,                    //   USAGE (Vendor Usage 1)同下同名解析
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)  同下同名解析
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255) 同下同名解析
    0x95, 0x40,                    //   REPORT_COUNT (64)　　同下REPORT_COUNT
    0x75, 0x08,                    //   REPORT_SIZE (8)　　　同下REPORT_SIZE
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)　表示USB要输入数据到PC的功能


    0x09, 0x01,                    //   USAGE (Vendor Usage 1) 每个功能的一个卷标志
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)    表示每个传输数据限定为0
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)    表示每个传输数据的最大值限定为255
    0x95, 0x40,                    //   REPORT_COUNT (64) 每次接收的数据长度，这里是64位
    0x75, 0x08,                    //   REPORT_SIZE (8)        传输字段的宽度为8bit，表示每个传输的数据范围为0~ffff ffff
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs) 表示USB设备要接收PC的数据的功能
    0xc0                           // END_COLLECTION　　结束标志
};
