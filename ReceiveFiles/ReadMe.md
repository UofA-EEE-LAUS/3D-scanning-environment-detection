# Files that receive data
All the files in this folder are used to receive the data that is trasnmitted from the Raspberry Pi to their respective platform.

# Receive into Matlab

This requires you to insert the IP address and port number thats established in the main file. You also need to write out how many characters are being transmitted by the Pi.

This is done by first figuring out how many characters the Raspberry Pi is transmitting by using a simple print statement in the main function.

# Receive into Simulink

Simulink is similar to MatLab, you just put in the IP address and port, and in the "data size" field, you insert "[1 (CharacterLength)]"
and it will display the received package on the right

# Receive into Arduino

Arduino receiving is relatively simple, it just works :

