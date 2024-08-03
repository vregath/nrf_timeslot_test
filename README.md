
### NRF radio timeslot and time sync test

The goal of this project is to test nrf52 sd_radio_session timeslot. It based on time sync demo from here:
github.com/nordic-auko/mRF5-ble-timesync-demo

For testing 2 nrf52 device and mobile with nrf connect installed is required. 

After installing demo connect to devices with nrf connect. 

Under Unknown Service (UUID: 00001000-1212-efde-1523-785feabc9844)
Unknow characteristic (UUID: 00002005-1212-efde-1523-785feabc9844)
you can write 3 command:

* 0xFF: start time sync as master (start radio session and start sending sync packets)
* 0xAA: start time sync as slave (start radio session and start to listen to sync packets)
* 0x11: stop radio session

Debug nrf52 device with JLink which will be slave. 

Send 0xFF to device which is not debugged. Send 0xAA to device which is debugged with JLink. 
If master device connects first to phone you should see with debugger:

[I]TS:call IRQ_handler

[I]TS:IRQ_handler

[I]TS:EVENTS_ADDRESS

[I]TS:EVENTS_PAYLOAD

[I]TS:EVENTS_PHYEND

[I]TS:EVENTS_SYNC

[I]TS:Sync packet received <xxx>

if not master device connect first disconnect from it. After that you should see that slave get packets. 
Connect again to master will stop receiving packets on slave device after some time (4-5 sec). 
if reconnect is done with slave receving packet should work.

If master device is debugged after starting as master log should print every case:

[I]TS:call IRQ_handler

[I]TS:IRQ_handler

[I]TS:EVENTS_ADDRESS

[I]TS:EVENTS_PAYLOAD

[I]TS:EVENTS_PHYEND

[I]TS:packet sent: 23

