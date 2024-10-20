#include "apple_picking_state_machine/progress_manager.hpp"
#include <indicators/dynamic_progress.hpp>
#include <indicators/progress_bar.hpp>
#include <algorithm>  // For std::min

// Define the hidden implementation in the source file
class ProgressManager::Impl {
public:
    Impl(const std::vector<std::string>& process_names, const std::vector<int>& timeouts):
        process_names_(process_names) {

        std::cout << "Creating ProgressManager with " << process_names.size() << " processes" << std::endl;

        if (process_names_.size() != timeouts.size()) 
            throw std::invalid_argument("Process names and timeouts must have the same size.");
        
        std::cout << "numb of processes: " << process_names_.size() << std::endl;
        start_times_.resize(process_names_.size());

        // Initialize progress bars and timeouts
        for (size_t i = 0; i < process_names_.size(); ++i) {
            auto bar = std::make_unique<indicators::ProgressBar>(   
                indicators::option::BarWidth{50},
                indicators::option::Start{"["},
                indicators::option::Fill{"■"},
                indicators::option::Lead{"■"},
                indicators::option::Remainder{" "},
                indicators::option::End{" ]"},
                indicators::option::ForegroundColor{indicators::Color::yellow},
                indicators::option::ShowElapsedTime{true},
                indicators::option::ShowRemainingTime{true},
                indicators::option::PrefixText{process_names_.at(i) + " "},
                indicators::option::MaxProgress{100}  // Set max progress to 100
            );
            
            dynamic_bars_.push_back(std::move(bar));  // Store progress bars
        }

        // Initialize timeouts vector
        timeouts_ = timeouts;  

        // Initialize DynamicProgress with the individual progress bars
        // bars_ = std::make_unique<indicators::DynamicProgress<indicators::ProgressBar>>(
        //     std::move(dynamic_bars_.at(0)), std::move(dynamic_bars_.at(1))  // Add more depending on your bars TODO: make this dynamic
        // );
        bars_ = std::make_unique<indicators::DynamicProgress<indicators::ProgressBar>>();
        for (auto& bar : dynamic_bars_) {
            bars_->push_back(std::move(bar));
        }

        bars_->set_option(indicators::option::HideBarWhenComplete{false});

    }

    // Updates the progress bars based on elapsed time using stored start times
    void update_progress(size_t index) {

        // std::cout << "Updating progress for process " << index << std::endl;

        auto current_time = std::chrono::steady_clock::now();        
        auto duration = std::chrono::duration<double>(current_time - start_times_.at(index)).count();

        std::cout << "Duration index : " << index << " : " << duration << std::endl;

        double progress = duration / timeouts_.at(index) * 100.0;
        progress = std::min(progress, 100.0);

        (*bars_)[index].set_progress(static_cast<size_t>(progress));
    }

    // Marks the given process as completed
    void complete_process(size_t index) {
        if (index >= dynamic_bars_.size()) {
            throw std::invalid_argument("Invalid index for process completion.");
        }
        (*bars_)[index].set_option(indicators::option::PrefixText{process_names_.at(index) + " [DONE] "});
        (*bars_)[index].mark_as_completed();
    }

    // Updates the start time for a specific progress bar
    void update_start_time(size_t index, const std::chrono::steady_clock::time_point& start_time) {
        start_times_.at(index) = start_time;
    }

private:
    std::vector<std::string> process_names_;  // Names of the processes
    std::vector<int> timeouts_;  // Corresponding timeouts for each progress bar
    std::unique_ptr<indicators::DynamicProgress<indicators::ProgressBar>> bars_;  // Dynamic progress manager
    std::vector<std::unique_ptr<indicators::ProgressBar>> dynamic_bars_;  // Dynamic list of bars
    std::vector<std::chrono::steady_clock::time_point> start_times_;  // Start times for each bar (only reserved)
};

// Constructor
ProgressManager::ProgressManager(const std::vector<std::string>& process_names, const std::vector<int>& timeouts)
    : impl_(std::make_unique<Impl>(process_names, timeouts)) {}

// Destructor
ProgressManager::~ProgressManager() = default;

// Update progress bars
void ProgressManager::update_progress(size_t index) {
    impl_->update_progress(index);
}

// Complete a process
void ProgressManager::complete_process(size_t index) {
    impl_->complete_process(index);
}

// // Add a new progress bar
// size_t ProgressManager::add_progress_bar(const std::string& name, int timeout) {
//     return impl_->add_progress_bar(name, timeout);
// }

// Update the start time for a specific bar
void ProgressManager::update_start_time(size_t index, const std::chrono::steady_clock::time_point& start_time) {
    impl_->update_start_time(index, start_time);
}