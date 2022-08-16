#coding=utf-8

import socket
import time
import traceback

def get_wifi_status():

    status="testing..."

    filename = '/mnt/udisk/check_wifi.cfg'

    file=open(filename,'r')         #打开目标文件

    for content in file.readlines():     #逐行读取
        if 'test_num' in content:
            test_num=content
        if 'success_num' in content:
            success_num=content
        if 'reboot_count' in content:
            reboot_count=content

    test_num = (int)(((test_num.split('='))[1].split('\n'))[0])
    success_num = (int)(((success_num.split('='))[1].split('\n'))[0])
    reboot_count = (int)(((reboot_count.split('='))[1].split('\n'))[0])

    if reboot_count == test_num:
        if success_num < reboot_count:
            status="error!"
        else:
            status="OK~"

    # print(test_num)
    # print(success_num)
    # print(reboot_count)
    # print(status)

    return status

