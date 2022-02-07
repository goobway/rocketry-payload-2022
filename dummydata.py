# generating dummy data

import random
import csv

line = 0

header = ['Acc', 'Mag', 'Gyro']

while line != 100:

    data = []
    for i in range(101):
        acc = random.randint(1, 20)
        mag = random.randint(1, 20)
        gyro = random.randint(1, 20)

        # print(acc, mag, gyro)

        reading = [acc, mag, gyro]
        data.append(reading)

    with open('data.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)

        # header
        writer.writerow(header)

        # rows
        writer.writerows(data)

    line += 1
