import os
import time
wifii = ["racecar_7"]
count = 1
version = 3
for i in wifii:
    os.makedirs(f"C:\\Users\\25wan\\Downloads\\racecar_code\\version_{version}\\team_{count}")

    os.system(f'cmd /c "netsh wlan connect name={i}"')
    print(i)
    time.sleep(10)
    os.system(f'''cmd /c "scp -r racecar@192.168.1.10{count}:/home/racecar/Documents /Users/25wan/Downloads/racecar_code/version_{version}/team_{count}\nracecar@mit"''')
    count +=1
# racecar@mit

#"racecar_1","racecar_2_5G","racecar_3", "racecar_4_5G","racecar_5","racecar_6",
