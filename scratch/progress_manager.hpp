#ifndef PROGRESS_MANAGER_HPP
#define PROGRESS_MANAGER_HPP

#include <indicators/dynamic_progress.hpp>
#include <indicators/progress_bar.hpp>
#include <vector>
#include <string>
#include <memory>
#include <chrono>

class ProgressManager {
public:
    // Constructor that takes vectors of process names and timeouts
    ProgressManager(const std::vector<std::string>& process_names, const std::vector<int>& timeouts);

    // Destructor
    ~ProgressManager();

    // Function to set the start time for a specific progress bar by its index
    void update_start_time(size_t index, const std::chrono::steady_clock::time_point& start_time);

    // Function to update all progress bars based on their elapsed time
    void update_progress(size_t index);

    // Marks the given process as completed by its index
    void complete_process(size_t index);

    // // Function to dynamically add a new progress bar with name and timeout
    // size_t add_progress_bar(const std::string& name, int timeout);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;  // Pimpl idiom for hiding implementation details
};

#endif // PROGRESS_MANAGER_HPP
