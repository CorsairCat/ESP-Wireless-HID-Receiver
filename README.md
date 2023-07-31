# ESP-Wireless-HID-Receiver
+ this receiver is based on the wireless and aim to provided 1000hz report rate
## This is Basic Receiver
+ current method is using pre-coded mac address to pair each other
+ no ack based boardcast
+ using the function of sniffer mode
## Basic Method
+ using boardcast + sniffer mode to create a physical level UDP, which can avoid the ACK and resend
## TO-DOs
+ add pair mode for the system
+ compare the method which use esp-now to keep the packet realiable