/*
 * Copyright [2015]
 * [Kartik Mohta <kartikmohta@gmail.com>]
 * [Ke Sun <sunke.polyu@gmail.com>]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mocap_vicon/ViconDriverComponent.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace mocap {

ViconDriverComponent::ViconDriverComponent(const rclcpp::NodeOptions & options)
    : Node("vicon_node", options), initialized_(false)
{
    RCLCPP_INFO(this->get_logger(), "Creating ViconDriverComponent...");
    
    // Defer initialization until after construction using a one-shot timer
    // This ensures shared_from_this() works properly
    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        [this]() {
            this->init();
            // Cancel the timer after initialization
            this->init_timer_->cancel();
            this->init_timer_.reset();
        }
    );
}

ViconDriverComponent::~ViconDriverComponent() {
    if (run_timer_) {
        run_timer_->cancel();
    }
    if (driver_) {
        driver_->disconnect();
    }
}

void ViconDriverComponent::init() {
    RCLCPP_INFO(this->get_logger(), "Initializing Vicon driver...");
    
    // Create the driver with shared_from_this()
    driver_ = std::make_unique<ViconDriver>(this->shared_from_this());
    
    if (!driver_->init()) {
        RCLCPP_ERROR(this->get_logger(), "Initialization of the Vicon driver failed");
        initialized_ = false;
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Successfully initialized Vicon connection!");
    initialized_ = true;
    
    // Start the run timer at a high frequency (e.g., 1ms)
    run_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        [this]() { this->runCallback(); }
    );
}

void ViconDriverComponent::runCallback() {
    if (initialized_ && driver_) {
        driver_->run();
    }
}

} // namespace mocap

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(mocap::ViconDriverComponent)
