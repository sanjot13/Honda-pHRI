"""Prints the readings of a Bota Systems EtherCAT sensor.

Usage: python bota_minimal_example.py <adapter>

This example expects a physical slave layout according to
_expected_slave_layout, see below.
"""

import sys
import struct
import time
import collections

import pysoem
import ctypes
import struct
import matplotlib.pyplot as plt
from numpy import asarray
import numpy as np
import serial

serial_port = '/dev/ttyACM0'
baud_rate = 9600
# ser = serial.Serial(
#     serial_port,
#     baud_rate,
#     bytesize=serial.EIGHTBITS,
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE,
#     timeout=1,
# )


class MinimalExample:

    BOTA_VENDOR_ID = 0xB07A
    BOTA_PRODUCT_CODE = 0x00000001
    SINC_LENGTH = 256
    # The time step is set according to the sinc filter size
    time_step = 1.0

    def __init__(self, ifname):
        self._ifname = ifname
        self._master = pysoem.Master()
        SlaveSet = collections.namedtuple(
            'SlaveSet', 'slave_name product_code config_func')
        self._expected_slave_mapping = {0: SlaveSet('BFT-MEDS-ECAT-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)}

    def bota_sensor_setup(self, slave_pos):
        print("bota_sensor_setup")
        slave = self._master.slaves[slave_pos]

        ## Set sensor configuration
        # calibration matrix active
        slave.sdo_write(0x8010, 1, bytes(ctypes.c_uint8(1)))
        # temperature compensation
        slave.sdo_write(0x8010, 2, bytes(ctypes.c_uint8(0)))
        # IMU active
        slave.sdo_write(0x8010, 3, bytes(ctypes.c_uint8(1)))

        ## Set force torque filter
        # FIR disable
        slave.sdo_write(0x8006, 2, bytes(ctypes.c_uint8(1)))
        # FAST enable
        slave.sdo_write(0x8006, 3, bytes(ctypes.c_uint8(0)))
        # CHOP enable
        slave.sdo_write(0x8006, 4, bytes(ctypes.c_uint8(0)))
        # Sinc filter size
        slave.sdo_write(0x8006, 1, bytes(ctypes.c_uint16(self.SINC_LENGTH)))

        ## Get sampling rate
        sampling_rate = struct.unpack('h', slave.sdo_read(0x8011, 0))[0]
        print("Sampling rate {}".format(sampling_rate))
        if sampling_rate > 0:
            self.time_step = 1.0/float(sampling_rate)

        print("time step {}".format(self.time_step))

    def run(self):

        self._master.open(self._ifname)

        # config_init returns the number of slaves found
        if self._master.config_init() > 0:

            print("{} slaves found and configured".format(
                len(self._master.slaves)))

            for i, slave in enumerate(self._master.slaves):
                assert(slave.man == self.BOTA_VENDOR_ID)
                assert(
                    slave.id == self._expected_slave_mapping[i].product_code)
                slave.config_func = self._expected_slave_mapping[i].config_func

            # PREOP_STATE to SAFEOP_STATE request - each slave's config_func is called
            self._master.config_map()

            # wait 50 ms for all slaves to reach SAFE_OP state
            if self._master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
                self._master.read_state()
                for slave in self._master.slaves:
                    if not slave.state == pysoem.SAFEOP_STATE:
                        print('{} did not reach SAFEOP state'.format(slave.name))
                        print('al status code {} ({})'.format(hex(slave.al_status),
                                                              pysoem.al_status_code_to_string(slave.al_status)))
                raise Exception('not all slaves reached SAFEOP state')

            self._master.state = pysoem.OP_STATE
            self._master.write_state()

            self._master.state_check(pysoem.OP_STATE, 50000)
            if self._master.state != pysoem.OP_STATE:
                self._master.read_state()
                for slave in self._master.slaves:
                    if not slave.state == pysoem.OP_STATE:
                        print('{} did not reach OP state'.format(slave.name))
                        print('al status code {} ({})'.format(hex(slave.al_status),
                                                              pysoem.al_status_code_to_string(slave.al_status)))
                raise Exception('not all slaves reached OP state')
            """
            Gather data
            """
            print("gathering data...")
            values_for_parsing = []
            count = 0
            bota_times = []
            bota_forces = []

            # change this to determine when to stop collecting data; code could be changed to collect for a certain amount of time as well
            BOTA_SAMPLING_TIME = 20
                # seconds
            num_values_to_plot = 200 * BOTA_SAMPLING_TIME
            try:
                while 1:
                    # free run cycle
                    self._master.send_processdata()
                    self._master.receive_processdata(2000)

                    sensor_input_as_bytes = self._master.slaves[0].input
                    status = struct.unpack_from('B', sensor_input_as_bytes, 0)[0]
                    print("Status {}".format(status))

                    warningsErrorsFatals = struct.unpack_from('I', sensor_input_as_bytes, 1)[0]
                    print("Warnings/Errors/Fatals {}".format(warningsErrorsFatals))

                    Fx = struct.unpack_from('f', sensor_input_as_bytes, 5)[0]
                    print("Fx {}".format(Fx))
                    Fy = struct.unpack_from('f', sensor_input_as_bytes, 9)[0]
                    print("Fy {}".format(Fy))
                    Fz = struct.unpack_from('f', sensor_input_as_bytes, 13)[0]
                    print("Fz {}".format(Fz))
                    Mx = struct.unpack_from('f', sensor_input_as_bytes, 17)[0]
                    print("Mx {}".format(Mx))
                    My = struct.unpack_from('f', sensor_input_as_bytes, 21)[0]
                    print("My {}".format(My))
                    Mz = struct.unpack_from('f', sensor_input_as_bytes, 25)[0]
                    print("Mz {}".format(Mz))
                    forceTorqueSaturated = struct.unpack_from('H', sensor_input_as_bytes, 29)[0]
                    print("Force-Torque saturated {}".format(forceTorqueSaturated))

                    Ax = struct.unpack_from('f', sensor_input_as_bytes, 31)[0]
                    print("Ax {}".format(Ax))
                    Ay = struct.unpack_from('f', sensor_input_as_bytes, 35)[0]
                    print("Ay {}".format(Ay))
                    Az = struct.unpack_from('f', sensor_input_as_bytes, 39)[0]
                    print("Az {}".format(Az))
                    accelerationSaturated = struct.unpack_from('B', sensor_input_as_bytes, 43)[0]
                    print("Acceleration saturated {}".format(accelerationSaturated))

                    Rx = struct.unpack_from('f', sensor_input_as_bytes, 44)[0]
                    print("Rx {}".format(Rx))
                    Ry = struct.unpack_from('f', sensor_input_as_bytes, 48)[0]
                    print("Ry {}".format(Ry))
                    Rz = struct.unpack_from('f', sensor_input_as_bytes, 52)[0]
                    print("Rz {}".format(Rz))
                    angularRateSaturated = struct.unpack_from('B', sensor_input_as_bytes, 56)[0]
                    print("Angular rate saturated {}".format(angularRateSaturated))

                    temperature = struct.unpack_from('f', sensor_input_as_bytes, 57)[0]
                    print("Temperature {}\n".format(temperature))
                    print("time {}\n".format(self.time_step))
                                        
                    """
                    Gather teensy data
                    """

                    # if ser.in_waiting:  # if there is data to be read
                    #     line = ser.readline()
                    #     read_line = line.decode("utf-8")
                    #     values_for_parsing.append(read_line)

                        # if len(values_for_parsing) >= num_values_to_plot:
                        #     break


                    count += 1
                    BOTA_SAMPLING_RATE = 1e2 / 2
                    bota_times.append(float(count) / 1e2 / 2)
                    bota_forces.append(float(Fz))
                    if count == num_values_to_plot:
                        """ 
                        Parse values into a 'times' array and a 'force array'
                        """
                        print("parsing values...")
                        times = []
                        forces = []

                        for values in values_for_parsing:
                            teensy_time, force = values.split(",")
                            times.append(float(teensy_time) / 1e6)
                            forces.append(float(force))


                        """
                        Plot Force vs. Time
                        do time conversion. = doneish
                        - drop out first 10 vals in teensy too
                        try getting teensy and bota in same .py

                        export bota and kinova to csv
                        post process and plot on one graph.

                        try to plot bota and kinova in the same script
                        - try kinova with .py instead of cpp
                        """
                        print("plotting...")
                        # offset times to start at zero
                        # times = asarray(times)
                        # times -= times[0]

                        bota_times = asarray(bota_times[10:])
                        bota_times -= bota_times[0]

                        bota_forces = -asarray(bota_forces[10:])
                        bota_forces -= bota_forces[0]
                        # write to csv 
                        zipped = zip(bota_times, bota_forces)
                        print(zipped)
                        np.savez('data_bota', x=bota_times, y=bota_forces)

                        # Save the array back to the file
                        # np.savetxt('z.csv', zipped, fmt='%i,%i')

                        # plot values
                        plt.plot(bota_times, bota_forces, 'r', lw=2, label="Bota Sensor")
                        # plt.plot(times, forces, 'b', lw=2)
                        plt.xlabel("Time (s)")
                        plt.ylabel(("Force (N)"))
                        plt.title(("Bota Force Sensor"))
                        
                        plt.show()

                    time.sleep(self.time_step)

            except KeyboardInterrupt:
                # ctrl-C abort handling
                print('stopped')

            self._master.state = pysoem.INIT_STATE
            # request INIT state for all slaves
            self._master.write_state()
            self._master.close()
        else:
            print('slaves not found')


if __name__ == '__main__':

    print('minimal_example')

    if len(sys.argv) > 1:
        try:
            MinimalExample(sys.argv[1]).run()
        except Exception as expt:
            print(expt)
            sys.exit(1)
    else:
        print('usage: minimal_example ifname')
        sys.exit(1)
