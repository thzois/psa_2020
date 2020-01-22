# Parallel System Architectures - UvA

## Run the code
1. make
2. ./lab*.bin tracefiles/trace_file_name.trf

Note: In the tracefiles the number near "p" is related to the number of processors. For the 1st lab use only the tracefiles with "p1".

## Lab 1: Implement a signle cache
Build a single 32kB 8-way set-associative L1 D-cache with a 32-Byte line size. The simulator must be able to be driven with the single processor trace files. The addresses in the trace files are 32bits. Assume a Memory latency of 100 cycles, and single cycle cache latency. Use a least-recently-used, write-back replacement strategy.

## Lab 2: Implement a multiprocessor coherent cache system

Simulate a multiprocessing system with a shared memory architecture. Your simulation should be able to support multiple processors, all working in parallel. Each of the processors is associated with a local cache, and all cache modules are connected to a single bus. A main memory is also connected to the bus, where all the correct data is supposed to be stored. Assume a pipelined memory which can handle one request per cycle and a latency of 100 cycles. In order to make cache data coherent within the system, implement a bus snooping protocol. This means that each cache controller connected to the bus receives every memory request on the bus made by other caches, and makes the proper modification on its local cache line state. You should implement the simplest VALID-INVALID protocol. Using your simulator you must perform experiments with different numbers of processors (1 to 8 processors) using the different trace files for each case. The bus can only serve one request at any certain time. Thus if one cache occupies the bus, the other cache has to wait for the current operation to complete before it can utilize the bus. The cache does not occupy the bus when it waits for the memory to respond. This means that you will need to implement a split-transaction bus. The responses from memory should have priority on the bus.
