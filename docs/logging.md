# Logging

A key requirement for the simulated system is that it will generate logs that can be displayed and analysed in the
application UI. There are a number of mechanisms for the generation and gathering of those logs but let us start with a
consideration of the amount of data.

Assume for now that the discussion is only around continuous black-box style data recording and that the only
requirement
is for the generation/storage of binary data.

## Data volumes.

Suppose that the robot generates a record of core variables and that it does so on every iteration of the system tick.
This is typically every millisecond so we have 1000 records per second.

If there is a live radio link, it is unlikely to be able to operate at more than 2Mbit/sec and a safer estimate might be
half that. Thus we might expect to be able to send up to 1000 bits every tick which amounts to around 120 bytes per
millisecond. For a bit of extra safety, call it 100 bytes per millisecond.

A binary record of 100 bytes can hold quite a lot of information. There must be a 4 byte time stamp and either the
records must all be fixed size or there must be some kind of delimiter. Fixed size records often mean that we cannot be
flexible in what is recorded and/or there will be some wasted space. For variable records, the delimiters, and possibly
some kind of error checking, will take up some more bytes. Allow 6 more bytes for delimiters and you get 90 bytes for
data. Some records may be longer, some may be smaller but the maximum average throughput cannot be more than 90 bytes
per
millisecond.

That is enough for 20 floats or 40 small integers or 90 bytes.

Plain text records should also be limited to 90 characters maximum. Just 80 characters should suffice, allowing some
extra bytes for overhead like line feeds, string lengths and so on.

## Storage

Streaming out data at 100,000 bytes per second would very quickly use up any on-board storage unless it were something
like an SD card. Even a 4Mbit Flash chip would be filled in 4-5 seconds.

Clearly, the data that is sent at systick rates will need to be in much smaller packets. Arguably the least that is
useful at that rate is a timestamp and the forward and rotational velocity. From these, a host application could
generate the full pose by integration. Rounding errors should not accumulate excessively over a typical logging period.
In binary, that might mean 12 bytes if the velocities are stored as floats or just 8 bytes if the velocities are stored
as 16-bit integers. With 8 byte, fixed-length records sent every millisecond, it would be possible to fill a 4Mbit Flash
chip in 64 seconds. This is enough for even a slow speed run and, ideally, several fast speed runs.

For better analysis though, at least four sensor readings are needed. There is sufficient accuracy in 8 bits of sensor
data for logging purposes so an additional 4 bytes could be added for a common four sensor design.

It is probably a good idea to add at least some state information to indicate the kind of trajectory currently being
used and maybe a few boolean flags. For example, wall edge detection might take up a couple of bits. Call that another 2
bytes.

That brings it all up to 14 bytes per record. For neatness, and to accommodate some future proofing or techniques like
COBS, it looks like a 16 byte record might be OK. One of those bytes might be taken up by a record type indicator so
that
different types of record can be stored in the same stream.

It rather looks like a minimally useful data record might consume 16 bytes of storage.

Now we are at 16000 bytes per second. This is well within the 100kbps budget of a radio link like Bluetooth - even at
230kbps - and would fill a 4Mbit Flash chip in 32 seconds. In an ideal world, that would be enough for all the speed
runs of a fast mouse or a couple of speed runs for a slower mouse. It would be best to be able to store at least two
runs so that there is a record of the last fully successful run and the last unsuccessful run.

## Smaller storage devices.

Unless here is a suitable flash chip, or SD card, connected to the robot, the on-board memory of the processor will have
to suffice. If you can scrape up 128k of RAM, you can store 8 seconds of data. Ideally, this is enough for a complete
speed run. A slower robot might need to record every other systick. That should be fine since slower robots may still
get the same spatial resolution within the maze. Depending on the processor chip in use, a 128K block of flash might be
available for non-volatile storage of the data.

With only 64K RAM available it is almost inevitable that the recording frequency will have to reduce or only part of a
run will be recorded.

## Data transmission

Most of the preceding discussion is about storage of binary records or their transmission over a radio link. Binary data
needs software to convert it into human readable form. When transferring that data to a PC for processing or storage, it
is much more useful for it to be as plain ASCII text.

To transmit the data live, it would be necessary to convert the 16 byte data packet into text lines of no more than 80
bytes. This is probably feasible though it needs a bit of work to handle worst-case outcomes.

Internally stored binary data records can be dumped as text with relative abandon since the throughput is not the main
concern.

It would be best if a single common text format for these records were defined. Such records would either be sent live
by the robot over a radio link or would be sent as the stored data is unpacked. If possible, line lengths of no more
than 64 bytes should be used.This allow some headroom for the transmission medium. Further, a 64 byte Serial port TX
buffer is a reasonable length to deal with. 
