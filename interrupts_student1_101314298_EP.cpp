/**
 * @file interrupts_student1_101314298.cpp
 * @author Sasisekhar Govind
 * @author Alex Rusu (101314298)
 * @brief template main.cpp file for Assignment 3 Part 1 of SYSC4001
 * 
 */


#include "interrupts_101265596_101314298.hpp"

void FCFS(std::vector<PCB> &ready_queue) 
{
    std::sort( 
                ready_queue.begin(),
                ready_queue.end(),
                []( const PCB &first, const PCB &second ){
                    return (first.arrival_time > second.arrival_time); 
                } 
            );
}

void schedule_EP(std::vector<PCB> &ready_queue) 
{
    // Remove any RUNNING processes from ready_queue
    ready_queue.erase(
        std::remove_if(ready_queue.begin(), ready_queue.end(),
                       [](PCB &p){ return p.state == RUNNING; }),
        ready_queue.end()
    );

    if (ready_queue.empty()) {return;}

    auto iterator = std::min_element(ready_queue.begin(), ready_queue.end(),
                               [](const PCB &a, const PCB &b) {
                                   return a.PID < b.PID; // lower PID = higher priority
                               });
    std::iter_swap(ready_queue.begin(), iterator);
}

// add std::string for bonus mark
std::tuple<std::string> run_simulation(std::vector<PCB> list_processes)
{
    std::vector<PCB> ready_queue;   // The ready queue of processes
    std::vector<PCB> wait_queue;    // The wait queue of processes
    std::vector<PCB> job_list;      // A list to keep track of all the processes

    unsigned int current_time = 0;
    PCB running;
    running.PID = -1;

    std::string execution_status = print_exec_header();

    // Main simulation loop
    do 
    {

        /////////////////////// ADD NEW PROCESSES //////////////////////
        // 1) Add newly arrived processes
        for (auto &process : list_processes) 
        {
            if (process.arrival_time == current_time) 
            {
                job_list.push_back(process);
                PCB &job = job_list.back();
                if (assign_memory(job)) 
                {
                    job.state = READY;
                    ready_queue.push_back(job);
                    execution_status += print_exec_status(current_time, job.PID, NEW, READY);
                } 
                else 
                {
                    job.state = NOT_ASSIGNED;
                }
            }
        }

        /////////////////////// MANAGE WAIT QUEUE ///////////////////////
        // 2) Move processes whose I/O is done
        for (int i = 0; i < wait_queue.size(); i++) 
        {
            PCB &p = wait_queue[i];
            if (current_time >= p.io_complete_time) 
            {
                execution_status += print_exec_status(current_time, p.PID, WAITING, READY);
                p.state = READY;
                ready_queue.push_back(p);
                sync_queue(job_list, p);
                wait_queue.erase(wait_queue.begin() + i);
                i--;
            }
        }

        ////////////////////////// SCHEDULER ////////////////////////////
        // 3) Schedule CPU if idle
        if (running.PID == -1 && !ready_queue.empty()) 
        {
            schedule_EP(ready_queue);
            run_process(running, job_list, ready_queue, current_time);
            execution_status += print_exec_status(current_time, running.PID, READY, RUNNING);
        }

        //////////////////////// RUNNING PROCESS ///////////////////////
        // 4) Running process logic
        if (running.PID != -1) 
        {
            running.remaining_time -= 1;  // consume one CPU tick
            int time_run = running.processing_time - running.remaining_time;

            // Trigger I/O AFTER this tick if multiple of io_freq
            if (running.io_freq != 0 &&
                time_run % running.io_freq == 0 &&
                running.remaining_time > 0)
            {
                execution_status += print_exec_status(current_time + 1, running.PID, RUNNING, WAITING);
                running.state = WAITING;

                // Restore original io_duration and set I/O completion time
                auto it = std::find_if(list_processes.begin(), list_processes.end(),
                                       [&](const PCB &proc){ return proc.PID == running.PID; });

                if (it != list_processes.end()) {running.io_duration = it->io_duration;}

                running.io_complete_time = current_time + 1 + running.io_duration; // done after io_duration ticks

                wait_queue.push_back(running);
                sync_queue(job_list, running);
                running.PID = -1; // CPU now idle
            }
            // Check termination AFTER CPU tick
            else if (running.remaining_time <= 0) 
            {
                execution_status += print_exec_status(current_time + 1, running.PID, RUNNING, TERMINATED);
                terminate_process(running, job_list);
                running.PID = -1;
            }
        }

        /////////////////////////// ADVANCE TIME ////////////////////////
        current_time++;

    } while (!ready_queue.empty() || !wait_queue.empty() || running.PID != -1 || !all_process_terminated(job_list));

    //////////////////////// CLOSE OUTPUT TABLE //////////////////////
    execution_status += print_exec_footer();
    return std::make_tuple(execution_status);
}

int main(int argc, char** argv) 
{

    /////////////////////////// INPUT FILE ///////////////////////////
    //Get the input file from the user
    if(argc != 2) {
        std::cout << "ERROR!\nExpected 1 argument, received " << argc - 1 << std::endl;
        std::cout << "To run the program, do: ./interrupts <your_input_file.txt>" << std::endl;
        return -1;
    }

    //Open the input file
    auto file_name = argv[1];
    std::ifstream input_file;
    input_file.open(file_name);

    //Ensure that the file actually opens
    if (!input_file.is_open()) {
        std::cerr << "Error: Unable to open file: " << file_name << std::endl;
        return -1;
    }

    ////////////////////////// PARSE INPUT ///////////////////////////
    //Parse the entire input file and populate a vector of PCBs.
    //To do so, the add_process() helper function is used (see include file).
    std::string line;
    std::vector<PCB> list_process;
    while (std::getline(input_file, line)) {
        if (line.empty()) continue;

        auto input_tokens = split_delim(line, ",");   
        for (auto &token : input_tokens) {
            // Remove leading/trailing whitespace (spaces, tabs, etc.)
            token.erase(0, token.find_first_not_of(" \t\r\n"));
            token.erase(token.find_last_not_of(" \t\r\n") + 1);
        }

        auto new_process = add_process(input_tokens);
        
        list_process.push_back(new_process);
    }

    input_file.close();



    ////////////////////////// RUN SIMULATION ////////////////////////
    //With the list of processes, run the simulation
    auto [exec] = run_simulation(list_process);

    write_output(exec, "execution.txt");

    return 0;
}
