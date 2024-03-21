import matplotlib.pyplot as plt
from numpy import asarray
import serial

serial_port = '/dev/ttyACM0'
baud_rate = 9600
ser = serial.Serial(
    serial_port,
    baud_rate,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1,
)

# change this to determine when to stop collecting data; code could be changed to collect for a certain amount of time as well
num_values_to_plot = 20000

"""
Gather data
"""
print("gathering data...")
values_for_parsing = []
while True:
    if ser.in_waiting:  # if there is data to be read
        line = ser.readline()
        read_line = line.decode("utf-8")
        values_for_parsing.append(read_line)

        if len(values_for_parsing) >= num_values_to_plot:
            break

""" 
Parse values into a 'times' array and a 'force array'
"""
print("parsing values...")
times = []
forces = []

for values in values_for_parsing:
    time, force = values.split(",")
    times.append(float(time) / 1e6)
    forces.append(float(force))

"""
Plot Force vs. Time
"""
print("plotting...")
# offset times to start at zero
times = asarray(times)
times -= times[0]

# plot values
plt.plot(times, forces, 'r', lw=2)
plt.xlabel("Time (s)")
plt.ylabel(("Force (N)"))
plt.show()
