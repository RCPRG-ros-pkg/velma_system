#!/usr/bin/env python

## Resets shared memory for IPC for Velma system.
# @ingroup utilities
# @file reset_shm_comm.py

import os

if __name__ == "__main__":

    shm_channels = [
        'EC_Command',
        'EC_Status',
        'lLwr_cmd',
        'lLwr_st',
        'rLwr_cmd',
        'rLwr_st',
        'velma_core_cmd',
        'velma_core_st',
        'velma_core_cs_ve_body_cmd',
        'velma_core_cs_ve_body_st',
        'velma_core_cmd2',
        'velma_core_st2',
    ]

    shm_files = []
    for channel_name in shm_channels:
        shm_files.append( '{}_data'.format(channel_name) )
        shm_files.append( '{}_hdr'.format(channel_name) )

    for file_name in shm_files:
        full_path = '/dev/shm/{}'.format(file_name)
        if os.path.exists(full_path):
            os.remove(full_path)
            print('Removed file "{}"'.format(full_path)) 
        else:
            print('The file "{}" does not exist'.format(full_path)) 

    print("Cleanup done.")
    print("You can now run the control system of Velma.")
