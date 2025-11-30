/**
 * @file interrupts_student1_101314298.cpp
 * @author Sasisekhar Govind
 * @author Alex Rusu (101314298)
 * @brief template main.cpp file for Assignment 3 Part 1 of SYSC4001
 * 
 */

#include<interrupts_student1_student2.hpp>

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
    ready_queue.erase( std::remove_if(ready_queue.begin(), ready_queue.end(),
                       [](PCB &p){ return p.state == RUNNING; }), ready_queue.end());

    if (ready_queue.empty()) { return;}

    // Lower PID = higher priority
    auto it = std::min_element(ready_queue.begin(), ready_queue.end(),
                               [](const PCB &a, const PCB &b) {
                                   return a.PID < b.PID;
                               });

    std::iter_swap(ready_queue.begin(), it);
}

std::tuple<std::string /* add std::string for bonus mark */> run_simulation(std::vector<PCB> list_processes) {

    std::vector<PCB> ready_queue;   //The ready queue of processes
    std::vector<PCB> wait_queue;    //The wait queue of processes
    std::vector<PCB> job_list;      //A list to keep track of all the processes.

    unsigned int current_time = 0;
    PCB running;

    //Initialize an empty running process
    idle_CPU(running);

    std::string execution_status;

    //make the output table (the header row)
    execution_status = print_exec_header();

    //Main simulation loop
    while(!all_process_terminated(job_list) || job_list.empty()) 
    {

        // 1) Populate the ready queue with processes as they arrive
        for(auto &process : list_processes) 
        {
            if(process.arrival_time == current_time) 
            {
                assign_memory(process);

                process.state = READY;
                ready_queue.push_back(process);
                job_list.push_back(process);

                execution_status += print_exec_status(current_time, process.PID, NEW, READY);
            }
        }

        ///////////////////////MANAGE WAIT QUEUE/////////////////////////

        for (int i = 0; i < wait_queue.size(); i++) 
        {
            PCB &p = wait_queue[i];
            p.io_duration -= 1;

            if (p.io_duration <= 0) 
            {
                execution_status += print_exec_status(current_time, p.PID, WAITING, READY);
                p.state = READY;
                ready_queue.push_back(p);
                sync_queue(job_list, p);
                wait_queue.erase(wait_queue.begin() + i);
                i--;
            }
        }

        //////////////////////////SCHEDULER//////////////////////////////

        //FCFS(ready_queue); //example of FCFS is shown here

        // Schedule CPU if idle
        if (running.PID == -1 && !ready_queue.empty()) 
        {
            schedule_EP(ready_queue);
            run_process(running, job_list, ready_queue, current_time);
            execution_status += print_exec_status(current_time, running.PID, READY, RUNNING);
        }

        // 4) Running process logic
        if (running.PID != -1) 
        {
            running.remaining_time -= 1; // consume 1 tick
            int time_run = running.processing_time - running.remaining_time;

            // I/O interrupt after tick if multiple of io_freq
            if (running.io_freq != 0 && time_run % running.io_freq == 0 && running.remaining_time > 0) 
            {
                execution_status += print_exec_status(current_time + 1, running.PID, RUNNING, WAITING);
                running.state = WAITING;

                // Restore original I/O duration
                auto iterator = std::find_if(list_processes.begin(), list_processes.end(),
                                       [&](const PCB &proc){ return proc.PID == running.PID; });

                if (iterator != list_processes.end()) {running.io_duration = iterator->io_duration;}

                wait_queue.push_back(running);
                sync_queue(job_list, running);
                idle_CPU(running); // CPU now idle
            } 
            // Check termination
            else if (running.remaining_time == 0) 
            {
                execution_status += print_exec_status(current_time + 1, running.PID, RUNNING, TERMINATED);
                terminate_process(running, job_list);
                idle_CPU(running);
            }
        }

        current_time++;
    }
    
    //Close the output table
    execution_status += print_exec_footer();

    return std::make_tuple(execution_status);
}


int main(int argc, char** argv) {

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

    //Parse the entire input file and populate a vector of PCBs.
    std::string line;
    std::vector<PCB> list_process;
    while(std::getline(input_file, line)) {
        auto input_tokens = split_delim(line, ", "); // handle ", " as delimiter
        auto new_process = add_process(input_tokens);
        list_process.push_back(new_process);
    }
    input_file.close();

    //With the list of processes, run the simulation
    auto [exec] = run_simulation(list_process);

    write_output(exec, "execution.txt");

    return 0;
}
