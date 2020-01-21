/*
 * File: lab2.cpp
 *
 * Author: Theodoros Zois [ UvA: 12491659 - VU: 2652652 ]
 * 
 * Implement a multiprocessor coherent cache system 
 * The code implements a write-through write-allocate cache
 * 
 */

#include <systemc>
#include <iostream>
#include <chrono>
#include <map>

#include "psa.h"

using namespace std;
using namespace chrono;
using namespace sc_core;

// See sc_main for more details
static bool nop_output_enabled; 
static bool show_locks;         
static int PENDING_CPUS;

class Bus_if: public virtual sc_interface {
    public:
        virtual int read(int cache_id, int address, const char* cache_name) = 0;
        virtual int write(int cache_id, int address, const char* cache_name) = 0;
        virtual int readX_start(int cache_id, int address, const char* cache_name) = 0;
        virtual int readX_complete(int cache_id, int address, const char* cache_name, int request_id) = 0;
};


// ====================== BUS START =====================
class BUS: public Bus_if, public sc_module
{
    public:
        enum BusOperation
        {
            PROBE_FUNC_READ,
            PROBE_FUNC_WRITE,
            PROBE_FUNC_READX
        };

        sc_in<bool>                                 Port_CLK;

        sc_port<sc_signal_inout_if<int>>            Port_BusAddr;
        sc_port<sc_signal_inout_if<int>>            Port_BusCacheID;
        sc_port<sc_signal_inout_if<BusOperation>>   Port_BusOp;

        // Metrics counters
        typedef struct 
        {
            unsigned int total_waits;
            unsigned int total_reads;
            unsigned int total_writes;
            unsigned int total_readsX;
        } metrics;

        metrics *caches_metrics;

        SC_CTOR(BUS) 
        { 
            sensitive << Port_CLK.pos();

            bus_locked = false;

            req_id = 0;

            caches_metrics = new metrics[PENDING_CPUS];
            for(int i = 0; i < PENDING_CPUS; i++)
            {
                caches_metrics[i].total_waits = 0;
                caches_metrics[i].total_reads = 0;
                caches_metrics[i].total_writes = 0;
                caches_metrics[i].total_readsX = 0;
            }
        }

    private:
        // Used to get exclusivity on the BUS
        bool bus_locked;
        sc_mutex bus_mutex;

        // Requests on the BUS are stored into a map
        typedef struct 
        {
            int address;
            BusOperation operation;
        } bus_request;

        unsigned int req_id;
        map<int, bus_request> requests_table;

        // Track if there are ready RAM responses
        int pending_ram_responses = 0;

        int cache_acquire_lock(int cache_id, const char* cache_name)
        {
            // Give priority to responses from RAM
            while(pending_ram_responses > 0)
            {
                caches_metrics[cache_id].total_waits++;
                wait();
            }

            while(bus_locked)
            {
                caches_metrics[cache_id].total_waits++;
                wait();
            }

            bus_locked = true;
            bus_mutex.lock();

            if(show_locks)
                cout << sc_time_stamp() << ": " << cache_name << " locked the BUS" << endl;

            return 0;
        }

        int ram_acquire_lock()
        {
            while(bus_locked)
                wait();

            bus_locked = true;
            bus_mutex.lock();

            if(show_locks)
                cout << sc_time_stamp() << ": RAM locked the BUS" << endl;

            return 0;
        }

        int release_lock(const char* module_name)
        {
            if(show_locks)
                cout << sc_time_stamp() << ": " << module_name << " unclocked the BUS" << endl;
                            
            bus_mutex.unlock();
            bus_locked = false;
            return 0;
        }

        virtual int read(int cache_id, int address, const char* cache_name) 
        { 
            cache_acquire_lock(cache_id, cache_name);

                // Increase the metric
                caches_metrics[cache_id].total_reads++;

                // Lock acquired - add entry to the requests_table and get its id    
                requests_table.insert(pair<int, bus_request> (req_id, { address, PROBE_FUNC_READ }));
                int cur_req_id = req_id;
                req_id++;   // Increase the id for the next request

                Port_BusAddr->write(address);
                Port_BusCacheID->write(cache_id);
                Port_BusOp->write(BUS::PROBE_FUNC_READ);
                cout << sc_time_stamp() << ": " << cache_name << " sends a PROBE_READ to the BUS for address: " << address << endl;

                wait(); // Wait for everyone snoop the bus

            release_lock(cache_name);
 


            // Fetch response from RAM
            wait(100);  // RAM latency
            pending_ram_responses++;

            ram_acquire_lock();

                cout << sc_time_stamp() << ": RAM response to " << cache_name << " for address: " << address << " - PROBE_READ" << endl;            

                // Delete entry from requests table and mark ram response as sent 
                requests_table.erase(cur_req_id);
                pending_ram_responses--;

            release_lock("RAM");

            return 0 ; 
        }

        virtual int write(int cache_id, int address, const char* cache_name)  
        {
            cache_acquire_lock(cache_id, cache_name);

                // Increase the metric
                caches_metrics[cache_id].total_writes++;

                // Lock acquired - add entry to the requests_table and get its index    
                requests_table.insert(pair<int, bus_request> (req_id, { address, PROBE_FUNC_WRITE }));

                Port_BusAddr->write(address);
                Port_BusCacheID->write(cache_id);
                Port_BusOp->write(BUS::PROBE_FUNC_WRITE);
                cout << sc_time_stamp() << ": " << cache_name << " sends a PROBE_WRITE to the BUS for address: " << address << endl;

                wait(); // Wait for everyone snoop the bus

                // Update RAM
                wait(100);

                // Remove entry from the requests table and increase the request id
                requests_table.erase(req_id);
                req_id++;
            release_lock(cache_name);

            return 0; 
        }

        virtual int readX_start(int cache_id, int address, const char* cache_name)
        {
            cache_acquire_lock(cache_id, cache_name);

                // Increase the metric
                caches_metrics[cache_id].total_readsX++;

                // Lock acquired - add entry to the requests_table and get its index    
                requests_table.insert(pair<int, bus_request> (req_id, { address, PROBE_FUNC_READX }));
                int cur_req_id = req_id;
                req_id++;   // Increase the request_id

                Port_BusAddr->write(address);
                Port_BusCacheID->write(cache_id);
                Port_BusOp->write(BUS::PROBE_FUNC_READX);
                cout << sc_time_stamp() << ": " << cache_name << " sends a PROBE_READX to the BUS for address: " << address << endl;

                wait(); // Wait for everyone snoop the bus

            release_lock(cache_name);



            // Fetch from RAM
            wait(100);  // RAM latency
            pending_ram_responses++;

            ram_acquire_lock();

                cout << sc_time_stamp() << ": RAM response to " << cache_name << " for address: " << address << " - PROBE_READX" << endl;            
                pending_ram_responses--;

            release_lock("RAM");

            return cur_req_id;
        } 

        virtual int readX_complete(int cache_id, int address, const char* cache_name, int request_id)
        {
            cache_acquire_lock(cache_id, cache_name);

                cout << sc_time_stamp() << ": " << cache_name << " updates address: " << address << " in RAM - PROBE_READX" << endl;
                
                // Update RAM
                wait(100);

                // Remove entry from the requests table and increase the request id
                requests_table.erase(request_id);

            release_lock(cache_name);

            return 0;
        }
};
// ====================== BUS END =====================



// ==================== CACHE START ===================
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

        sc_in<bool>                 Port_CLK;
        sc_in<Function>             Port_Func;
        sc_in<int>                  Port_Addr;
        sc_out<RetCode>             Port_Done;
        sc_inout_rv<32>             Port_Data;

        sc_port<sc_signal_in_if<int>>               Port_BusAddr;       // read address from the BUS
        sc_port<sc_signal_in_if<int>>               Port_BusCacheID;    // CACHE_ID that writes to the BUS 
        sc_port<sc_signal_in_if<BUS::BusOperation>> Port_BusOp;         // BUS operations (events)
        sc_port<Bus_if>                             Port_Bus;           // connect cache with the BUS

        int CACHE_ID;
        bool snooping_enabled; 

        SC_CTOR(L1_Cache)
        {
            // Create threads
            SC_THREAD(bus_handler);     // Listen to the bus
            sensitive << Port_CLK.pos();

            SC_THREAD(execute);         // Handle data
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

    private:
        static const int CACHE_SIZE = 32768;    // 32KB
        static const int BLOCK_SIZE = 32;       // 32B 
        static const int NUM_OF_WAYS = 8;       // 8-way
        static const int CACHE_LINES = CACHE_SIZE/BLOCK_SIZE;
        static const int NUM_OF_SETS = CACHE_LINES/NUM_OF_WAYS;

        // Cache way
        typedef struct {
            unsigned int lru;
            unsigned int valid:1;

            unsigned int tag:20;       
        } cache_way;

        // Cache set (consists of many ways)
        typedef struct {
            cache_way ways[NUM_OF_WAYS];
        } cache_set;

        // Cache (consists of many sets)
        typedef struct {
            cache_set sets[NUM_OF_SETS];
        } l1_cache;

        l1_cache *cache;

        // 8-way  -> 7 bits for set index
        //        -> 5 bits for block offset
        //        -> 20 bits for tag
        int get_tag(int address)
        {
            return address >> 12;
        }

        int get_set_index(int address)
        {
            return (address & 0x00000FE0) >> 5;
        }



        int get_lru(int set_index)
        {
            // If an entry exists with invalid data return it
            for(int i = 0; i < NUM_OF_WAYS; i++)
            {
                if(cache->sets[set_index].ways[i].valid == 0)
                    return i;
            }

            // If it does not exist, there will always be a value 0 for lru
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
            unsigned int tag  = get_tag(address);
            unsigned int set_index = get_set_index(address);
            
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
                            cout << sc_time_stamp() << ": " << name() << " HIT (Read) address: " << address << " tag: " << tag << " set: " << set_index << " way: " << i << endl;
                            stats_readhit(CACHE_ID);
                            update_lru(set_index, i);
                            found = true;

                            wait(); // Serve CPU
                            break;
                        }
                    }
                }

                if(!found)
                {
                    int way_idx = get_lru(set_index);

                    // Cache ReadMiss
                    cout << sc_time_stamp() << ": " << name() << " MISS (Read) address: " << address << " tag: " << tag << " set: " << set_index << " - FETCH from RAM" << endl;
                    stats_readmiss(CACHE_ID);

                    // Fetch from RAM - PROBE_READ
                    Port_Bus->read(CACHE_ID, address, name());

                    // Insert into cache
                    cache->sets[set_index].ways[way_idx].tag = tag;
                    cache->sets[set_index].ways[way_idx].valid = 1;
                    update_lru(set_index, way_idx);
                    
                    wait(); // Serve CPU
                }
            }
            else if(type == FUNC_WRITE)
            {
                for(int i = 0; i < NUM_OF_WAYS; i++)
                {
                    if(cache->sets[set_index].ways[i].tag == tag)
                    {
                        // Cache WriteHit
                        cout << sc_time_stamp() << ": " << name() << " HIT (Write) address: " << address << " tag: " << tag << " set: " << set_index << " way: " << i << " - UPDATE RAM" << endl;
                        stats_writehit(CACHE_ID);

                        cache->sets[set_index].ways[i].valid = 1;
                        update_lru(set_index, i);
                        found = true;    
                        wait(); // Serve CPU

                        // Update RAM - PROBE_WRITE
                        Port_Bus->write(CACHE_ID, address, name());
                        break;
                    }
                }

                if(!found)
                {
                    // Cache WriteMiss
                    cout << sc_time_stamp() << ": " << name() << " MISS (Write) address: " << address << " tag: " << tag << " set: " << set_index << " - FETCH from RAM" << endl;
                    stats_writemiss(CACHE_ID);

                    // Fetch from RAM - PROBE_READX
                    int request_id = Port_Bus->readX_start(CACHE_ID, address, name());

                    int way_idx = get_lru(set_index);
                    cache->sets[set_index].ways[way_idx].tag = tag;
                    cache->sets[set_index].ways[way_idx].valid = 1;
                    update_lru(set_index, way_idx);
                    wait(); // Serve CPU

                    // Update RAM
                    Port_Bus->readX_complete(CACHE_ID, address, name(), request_id);
                }
            }

            // Return data
            return 0;
        }

        void invalidate_copy(int address)
        {
            unsigned int tag  = get_tag(address);
            unsigned int set_index = get_set_index(address);

            for(int i = 0; i < NUM_OF_WAYS; i++)
            {   
                if(cache->sets[set_index].ways[i].valid == 1 && cache->sets[set_index].ways[i].tag == tag)
                {
                    cache->sets[set_index].ways[i].valid = 0;
                    cout << sc_time_stamp() << ": " << name() << " invalidated copy of address: " << address << " tag: " << tag << " set: " << set_index << " way: " << i << endl;
                }
            }
        }

        void bus_handler()
        {
            while(snooping_enabled)
            {
                wait(Port_BusOp->value_changed_event());
                BUS::BusOperation r = Port_BusOp->read();
                int cache_id_req = Port_BusCacheID->read();
                int address = Port_BusAddr->read();
                
                if(r == BUS::PROBE_FUNC_READ)
                {
                    // The caches that receive FUNC_READ they do nothing in practise
                    // A cache ignores its own request
                    if(cache_id_req != CACHE_ID)
                        cout << sc_time_stamp() << ": " << name() << " snooping PROBE_READ sent on the BUS from CACHE_"  << cache_id_req << " for address: " << address << endl;
                }
                else if(r == BUS::PROBE_FUNC_WRITE)
                {
                    // All the writes include invalidate_copy (either WriteMiss or WriteHit)
                    // A cache ignores its own request
                    if(cache_id_req != CACHE_ID)
                    {
                        cout << sc_time_stamp() << ": " << name() << " snooping PROBE_WRITE sent on the BUS from CACHE_"  << cache_id_req << " for address: " << address << endl;
                        invalidate_copy(address);
                    }
                }
                else if(r == BUS::PROBE_FUNC_READX)
                {
                    // All the readX include invalidate_copy (either WriteMiss or WriteHit)
                    // A cache ignores its own request
                    if(cache_id_req != CACHE_ID)
                    {
                        cout << sc_time_stamp() << ": " << name() << " snooping PROBE_READX sent on the BUS from CACHE_"  << cache_id_req << " for address: " << address << endl;
                        invalidate_copy(address);
                    }
                }
                else
                {
                    cout << "ERROR in " << __FUNCTION__ << ": Function does not exist " << endl;
                }
            }
        }

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
                    Port_Data.write("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
                }
                else if (f == FUNC_WRITE)
                {
                    data = search_cache(addr, FUNC_WRITE);
                    Port_Done.write( RET_WRITE_DONE );
                }
            }
        }
};
// ==================== CACHE END ===================



// ==================== CPU START ===================
SC_MODULE(CPU)
{
    public:
        sc_in<bool>                     Port_CLK;
        sc_in<L1_Cache::RetCode>        Port_CacheDone;
        sc_out<L1_Cache::Function>      Port_CacheFunc;
        sc_out<int>                     Port_CacheAddr;
        sc_inout_rv<32>                 Port_CacheData;

        int CPU_ID;

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
                if(!tracefile_ptr->next(CPU_ID, tr_data))
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
                    int address = tr_data.addr;
                    Port_CacheAddr.write(address);
                    Port_CacheFunc.write(f);

                    if (f == L1_Cache::FUNC_WRITE)
                    {
                        cout << sc_time_stamp() << ": " << name() << " sends write for address: " << address << endl;

                        uint32_t data = rand();
                        Port_CacheData.write(data);
                        wait();
                        Port_CacheData.write("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
                    }
                    else
                    {
                        cout << sc_time_stamp() << ": " << name() << " sends read for address: " << address << endl;
                    }

                    wait(Port_CacheDone.value_changed_event());

                    if (f == L1_Cache::FUNC_READ)
                    {
                        cout << sc_time_stamp() << ": " << name() << " reads: " << Port_CacheData.read() << endl;
                    }
                }
                else
                {
                    if(nop_output_enabled)
                        cout << sc_time_stamp() << ": " << name() << " executes NOP" << endl;
                }
                // Advance one cycle in simulated time
                wait();
            }

            // If all CPUs have finished, stop the simulation
            PENDING_CPUS--;
            if(PENDING_CPUS == 0)
                sc_stop();
        }
};
// ==================== CPU END ===================



int sc_main(int argc, char* argv[])
{
    try
    {
        setenv("SC_SIGNAL_WRITE_CHECK", "DISABLE", 1);
		sc_get_curr_simcontext()->reset();

        // Get the tracefile argument and create Tracefile object
        // This function sets tracefile_ptr and num_cpus
        init_tracefile(&argc, &argv);

        nop_output_enabled = false; // Disabling NOP messagess speeds up testing (stdout is overloaded)
        show_locks = true;          // lock/unlock messages to stdout
        PENDING_CPUS = num_cpus;

        // Initialize statistics counters
        stats_init();

        // The clock that will drive the CPU, Cache and the BUS
        sc_clock clk;
        
        // CPU - Cache signals/buffers declaration
        sc_buffer<L1_Cache::Function>   sigCacheFunc[num_cpus];
        sc_buffer<L1_Cache::RetCode>    sigCacheDone[num_cpus];
        sc_signal<int>                  sigCacheAddr[num_cpus];
        sc_signal_rv<32>                sigCacheData[num_cpus];

        /* CHECK */ // Maybe buffers for the others?
        // BUS - Cache signals/buffers
        sc_signal<int>                  sigBusCacheID;
        sc_signal<int>                  sigBusAddr;
        sc_buffer<BUS::BusOperation>    sigBusOp;

        L1_Cache* cache[num_cpus];
        
        CPU* cpu[num_cpus];

        BUS bus("BUS");
        bus.Port_CLK(clk);
        bus.Port_BusOp(sigBusOp);
        bus.Port_BusAddr(sigBusAddr);
        bus.Port_BusCacheID(sigBusCacheID);

        for(int i = 0; i < num_cpus; i++)
        {
            // Instantiate L1_Cache modules
            // Set a unique name (cache_ID)
            // Set CACHE_ID
            // Set snooping_enabled 
            char cache_name[10];
            sprintf(cache_name, "CACHE_%d", i);
            cache[i] = new L1_Cache(cache_name);
            cache[i]->CACHE_ID = i;
            cache[i]->snooping_enabled = true;

            // Connect Cache to CPU signals and clock
            cache[i]->Port_Func(sigCacheFunc[i]);
            cache[i]->Port_Addr(sigCacheAddr[i]);
            cache[i]->Port_Data(sigCacheData[i]);
            cache[i]->Port_Done(sigCacheDone[i]);
            cache[i]->Port_CLK(clk); 


            // Connect Cache to BUS signals
            cache[i]->Port_Bus(bus);
            cache[i]->Port_BusAddr(sigBusAddr);
            cache[i]->Port_BusOp(sigBusOp);
            cache[i]->Port_BusCacheID(sigBusCacheID);


            // Instantiate CPU modules
            // Set a unique name (cpu_ID)
            // Set CPU_ID
            char cpu_name[10];
            sprintf(cpu_name, "CPU_%d", i);
            cpu[i] = new CPU(cpu_name);
            cpu[i]->CPU_ID = i;

            // Connect CPU to Cache signals and clock
            cpu[i]->Port_CacheFunc(sigCacheFunc[i]);
            cpu[i]->Port_CacheAddr(sigCacheAddr[i]);
            cpu[i]->Port_CacheData(sigCacheData[i]);
            cpu[i]->Port_CacheDone(sigCacheDone[i]);
            cpu[i]->Port_CLK(clk); 
        }

        cout << "Running (press CTRL+C to interrupt)... " << endl;


        // Start Simulation - Measure execution time
        steady_clock::time_point begin = high_resolution_clock::now();
        sc_start();
        steady_clock::time_point end = high_resolution_clock::now();
        sc_time total_sys_time = sc_time_stamp();

        // Print statistics after simulation finished
        stats_print();
        stats_cleanup();

        cout << endl << "Main memory access rates" << endl;
        // cout << "  - Total waits for the BUS: " << total_waits << endl;
        // cout << "  - Average waiting time per access: " << (double)total_waits / double(total_reads + total_writes + total_readsX) << " cycles" << endl;
        
        unsigned int total_waits = 0;
        for(int i = 0; i < num_cpus; i++)
        {
            total_waits += bus.caches_metrics[i].total_waits;
        }

        cout << endl << "Average time for BUS acquisition (caches) " << double(total_waits) / double(num_cpus) << " cycles" << endl;

        cout << endl << "Total simulation time (sys) " << total_sys_time << endl;
        cout << "Total simulation time (real) " << duration_cast<duration<double>>(end - begin).count() << " sec" << endl;

        if(show_locks)
            cout << endl << "Lock/Unlock messages are enabled" << endl;

        if(!nop_output_enabled)
            cout << endl << "NOP messages for each CPU are disabled in order to speedup the process of testing" << endl;
    }
    catch (exception& e)
    {
        cerr << e.what() << endl;
    }

    return 0;
}
