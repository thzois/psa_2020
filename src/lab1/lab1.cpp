/*
 * File: assignment1.cpp
 *
 * Framework to implement Task 1 of the Advances in Computer Architecture lab
 * session. This uses the framework library to interface with tracefiles which
 * will drive the read/write requests
 *
 * Author(s): Michiel W. van Tol, Mike Lankamp, Jony Zhang,
 *            Konstantinos Bousias
 * Copyright (C) 2005-2017 by Computer Systems Architecture group,
 *                            University of Amsterdam
 *
 * 
 * 8-Way Set Associative Cache 
 * Theodoros Zois [ UvA: 12491659 - VU: 2652652 ]
 */

#include <systemc>
#include <iostream>

#include "psa.h"

using namespace std;
using namespace sc_core; // This pollutes namespace, better: only import what you need.

static const int CACHE_SIZE = 32768;    // 32KB
static const int BLOCK_SIZE = 32;       // 32B 
static const int NUM_OF_WAYS = 8;       // n-way
static const int CACHE_LINES = CACHE_SIZE/BLOCK_SIZE;
static const int NUM_OF_SETS = CACHE_LINES/NUM_OF_WAYS;


SC_MODULE(L1_Cache)
{
    public:
        enum Function 
        {
            FUNC_READ,
            FUNC_WRITE
        };

        enum RetCode 
        {
            RET_READ_DONE,
            RET_WRITE_DONE,
        };

        sc_in<bool>     Port_CLK;
        sc_in<Function> Port_Func;
        sc_in<int>      Port_Addr;
        sc_out<RetCode> Port_Done;
        sc_inout_rv<32> Port_Data;

        // Cache way
        typedef struct {
            unsigned int lru;
            unsigned int valid:1;
            unsigned int dirty:1;

            unsigned int tag:20;
            // + data but we don't need them now            
        } cache_way;

        // Cache set (consists of many ways)
        typedef struct {
            cache_way ways[NUM_OF_WAYS];
        } cache_set;

        // Cache (consists of many sets)
        typedef struct {
            cache_set sets[NUM_OF_SETS];
        } l1_cache;

        SC_CTOR(L1_Cache)
        {
            SC_THREAD(execute);
            sensitive << Port_CLK.pos();
            dont_initialize();

            cache = new l1_cache;
            int lru_counter = 0;

            // Initialize valid bits to zero and the LRU counter
            // Also tag to -1 unsigned (by default its initialized to zero and causes more cache hits)
            for(int i = 0; i < NUM_OF_SETS; i++)
            {
                for(int j = 0; j < NUM_OF_WAYS; j++)
                {
                    cache->sets[i].ways[j].valid = 0;
                    cache->sets[i].ways[j].lru = lru_counter;

                    cache->sets[i].ways[j].tag = -1;   
                    lru_counter++;

                    if(lru_counter > NUM_OF_WAYS - 1)
                        lru_counter = 0;
                }
            }
        }

        ~L1_Cache()
        {
            delete cache;
        }

        int get_lru(int set_index)
        {
            // There will always be a value 0
            for(int i = 0; i < NUM_OF_WAYS; i++)
            {
                if(cache->sets[set_index].ways[i].lru == 0)
                    return i;
            }

            return -1;
        }

        void update_lru(int set_index, int way_idx)
        {
            // If the MRU is accessed again, we don't have to update anything
            if(cache->sets[set_index].ways[way_idx].lru == NUM_OF_WAYS - 1)
                return;


            // The block that got accessed is not the LRU nor the MRU
            if(cache->sets[set_index].ways[way_idx].lru > 0)
            {
                // Update the counters - decrease their value by one
                // Only for those that are above the acounter of the accessed block
                for(int i = 0; i < NUM_OF_WAYS; i++)
                {
                    if(cache->sets[set_index].ways[i].lru > cache->sets[set_index].ways[way_idx].lru)
                        cache->sets[set_index].ways[i].lru = cache->sets[set_index].ways[i].lru - 1;
                }
            }
            else
            {
                // The block that got accessed is the LRU (0)
                // Update the counters - decrease their value by one
                for(int i = 0; i < NUM_OF_WAYS; i++)
                {
                    cache->sets[set_index].ways[i].lru = cache->sets[set_index].ways[i].lru - 1;
                }
            }

            // Make the accessed block the MRU
            cache->sets[set_index].ways[way_idx].lru = NUM_OF_WAYS - 1;
        }

        int search_cache(int address, Function type)
        {
            // 8-way  -> 7 bits for set index
            //        -> 5 bits for block offset
            //        -> 20 bits for tag
            unsigned int tag  = address >> 12;
            unsigned int set_index = (address & 0x00000FE0) >> 5;

            if(NUM_OF_WAYS == 2)
            {
                // 2-way -> 9 bits for set index
                //       -> 5 bits for block offset
                //       -> 18 bits for tag
                tag  = address >> 14;
                set_index = (address & 0x00001FF0) >> 5;
            }
            else if(NUM_OF_WAYS == 4)
            {
                // 4-way -> 8 bits for set index
                //       -> 5 bits for block offset
                //       -> 19 bits for tag
                tag  = address >> 13;
                set_index = (address & 0x00000FF0) >> 5;
            }
            else if(NUM_OF_WAYS == 16)
            {
                // 16-way -> 6 bits for set index
                //        -> 5 bits for block offset
                //        -> 21 bits for tag
                tag  = address >> 11;
                set_index = (address & 0x000007E0) >> 5;
            }
            
            int found = false;

            if(type == FUNC_READ)
            {
                // Determine if its a hit or miss
                for(int i = 0; i < NUM_OF_WAYS; i++)
                {
                    if(cache->sets[set_index].ways[i].valid == 1)
                    {
                        if(cache->sets[set_index].ways[i].tag == tag)
                        {
                            // Cache ReadHit
                            cout << sc_time_stamp() << ": CACHE HIT (Read) address: " << address << " tag: " << tag << " set: " << set_index << " way: " << i << endl;
                            stats_readhit(0);
                            wait();

                            found = true;

                            // Update LRU
                            update_lru(set_index, i);
                            break;
                        }
                    }
                }

                if(!found)
                {
                    int way_idx = get_lru(set_index);
                    if(cache->sets[set_index].ways[way_idx].dirty == 1)
                    {
                        // Update the dirty data in the main memory
                        cout << sc_time_stamp() << ": CACHE MISS (Read) address: " << address << " tag: " << tag << " set: " << set_index << " - WRITE-BACK & fetch from RAM" << endl;
                        stats_readmiss(0);
                        wait(200);
                    }
                    else
                    {
                        // Cache ReadMiss
                        cout << sc_time_stamp() << ": CACHE MISS (Read) address: " << address << " tag: " << tag << " set: " << set_index << " - Fetch from RAM" << endl;
                        stats_readmiss(0);
                        wait(100);
                    }

                    cache->sets[set_index].ways[way_idx].tag = tag;
                    cache->sets[set_index].ways[way_idx].valid = 1;
                    cache->sets[set_index].ways[way_idx].dirty = 0;
                    update_lru(set_index, way_idx);
                }
            }
            else if(type == FUNC_WRITE)
            {
                for(int i = 0; i < NUM_OF_WAYS; i++)
                {
                    if(cache->sets[set_index].ways[i].tag == tag)
                    {
                        // Cache WriteHit
                        cout << sc_time_stamp() << ": CACHE HIT (Write) address: " << address << " tag: " << tag << " set: " << set_index << " way: " << i << " - DIRTY = 1" << endl;
                        stats_writehit(0);
                        wait();

                        found = true;

                        cache->sets[set_index].ways[i].dirty = 1;
                        update_lru(set_index, i);
                        break;
                    }
                }

                if(!found)
                {
                    // Cache WriteMiss
                    cout << sc_time_stamp() << ": CACHE MISS (Write) address: " << address << " tag: " << tag << " set: " << set_index << " - Fetch from RAM" << endl;
                    stats_writemiss(0);
                    wait(100);

                    int way_idx = get_lru(set_index);
                    cache->sets[set_index].ways[way_idx].tag = tag;
                    cache->sets[set_index].ways[way_idx].valid = 1;
                    update_lru(set_index, way_idx);
                }
            }

            // Return data
            return 0;
        }

    private:
        l1_cache *cache;

        void execute() 
        {
            while (true)
            {
                wait(Port_Func.value_changed_event());
                Function f = Port_Func.read();
                int addr   = Port_Addr.read();
                int data   = 0;

                if (f == FUNC_READ) 
                {
                    data = search_cache(addr, FUNC_READ);
                    Port_Data.write(data);
                    Port_Done.write( RET_READ_DONE );
                    wait();
                    Port_Data.write("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
                }
                else
                {
                    data = search_cache(addr, FUNC_WRITE);
                    Port_Done.write( RET_WRITE_DONE );
                }
            }
        }
};


SC_MODULE(CPU)
{
    public:
        sc_in<bool>                     Port_CLK;
        sc_in<L1_Cache::RetCode>        Port_CacheDone;
        sc_out<L1_Cache::Function>      Port_CacheFunc;
        sc_out<int>                     Port_CacheAddr;
        sc_inout_rv<32>                 Port_CacheData;

        SC_CTOR(CPU)
        {
            SC_THREAD(execute);
            sensitive << Port_CLK.pos();
            dont_initialize();
        }

    private:
        void execute()
        {
            TraceFile::Entry tr_data;
            L1_Cache::Function f;

            // Loop until end of tracefile
            while(!tracefile_ptr->eof())
            {
                // Get the next action for the processor in the trace
                if(!tracefile_ptr->next(0, tr_data))
                {
                    cerr << "Error reading trace for CPU" << endl;
                    break;
                }

                switch(tr_data.type)
                {
                    case TraceFile::ENTRY_TYPE_READ:
                        f = L1_Cache::FUNC_READ;
                        break;

                    case TraceFile::ENTRY_TYPE_WRITE:
                        f = L1_Cache::FUNC_WRITE;
                        break;

                    case TraceFile::ENTRY_TYPE_NOP:
                        break;

                    default:
                        cerr << "Error, got invalid data from Trace" << endl;
                        exit(0);
                }

                if(tr_data.type != TraceFile::ENTRY_TYPE_NOP)
                {
                    Port_CacheAddr.write(tr_data.addr);
                    Port_CacheFunc.write(f);

                    if (f == L1_Cache::FUNC_WRITE)
                    {
                        cout << sc_time_stamp() << ": CPU sends write" << endl;

                        uint32_t data = rand();
                        Port_CacheData.write(data);
                        wait();
                        Port_CacheData.write("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
                    }
                    else
                    {
                        cout << sc_time_stamp() << ": CPU sends read" << endl;
                    }

                    wait(Port_CacheDone.value_changed_event());

                    if (f == L1_Cache::FUNC_READ)
                    {
                        cout << sc_time_stamp() << ": CPU reads: " << Port_CacheData.read() << endl;
                    }
                }
                else
                {
                    cout << sc_time_stamp() << ": CPU executes NOP" << endl;
                }
                // Advance one cycle in simulated time
                wait();
            }

            // Finished the Tracefile, now stop the simulation
            sc_stop();
        }
};


int sc_main(int argc, char* argv[])
{
    try
    {
        // Get the tracefile argument and create Tracefile object
        // This function sets tracefile_ptr and num_cpus
        init_tracefile(&argc, &argv);

        // Initialize statistics counters
        stats_init();

        // Instantiate Modules
        L1_Cache cache("l1_cache");
        CPU cpu("cpu");

        // Signals
        sc_buffer<L1_Cache::Function>   sigCacheFunc;
        sc_buffer<L1_Cache::RetCode>    sigCacheDone;
        sc_signal<int>                  sigCacheAddr;
        sc_signal_rv<32>                sigCacheData;

        // The clock that will drive the CPU and Cache
        sc_clock clk;

        // Connecting module ports with signals
        cache.Port_Func(sigCacheFunc);
        cache.Port_Addr(sigCacheAddr);
        cache.Port_Data(sigCacheData);
        cache.Port_Done(sigCacheDone);

        cpu.Port_CacheFunc(sigCacheFunc);
        cpu.Port_CacheAddr(sigCacheAddr);
        cpu.Port_CacheData(sigCacheData);
        cpu.Port_CacheDone(sigCacheDone);

        cache.Port_CLK(clk);
        cpu.Port_CLK(clk);

        cout << "Running (press CTRL+C to interrupt)... " << endl;

        // Start Simulation
        sc_start();

        // Print statistics after simulation finished
        stats_print();
    }
    catch (exception& e)
    {
        cerr << e.what() << endl;
    }

    return 0;
}
